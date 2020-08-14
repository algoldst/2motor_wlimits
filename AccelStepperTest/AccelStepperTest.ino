/* Author: Alex Goldstein
 *  
 * Platform: Smraza Arduino UNO. 
 *           Adafruit Motorshield v2.3
 * 
 * Software: Arduino IDE v1.8.13
 * 
 * Libraries: Adafruit Motorshield v2. Adafruit PWM Servo. Accelstepper. Arduino-timer. ElapsedMillis.
 * These can all be installed via the IDE's "Manage Libraries".
 * 
 * Description: 
 * This code controls an Arduino with Motorshield to run two bipolar stepper motors. Under typical operation, 
 * the motors extend/retract their attached loads (in this case, boom arms) until full extension/retraction 
 * is reached. This is determined either by limit switches or when the software's estimated motor positions 
 * reach their targets. 
 * 
 * Targets are set by constants `boom1StepLength` and `boom2StepLength`, respectively.
 * 
 * Limit switches 1 & 3 correspond to Motor 1 to indicate full extension/retraction. Switches 2 & 4 act in the same fashion for Motor 2.
 * 
 * Activity is determined according to a Finite State Machine (FSM). States: {IN, MOVE_OUT, OUT, MOVE_IN}
 * The FSM begins in the "IN" state with both motors at rest. When the Arduino receives a logic HIGH on its 
 * HID Advance State pin, it advances to MOVE_OUT. Both motors begin turning until full extension is determined,
 * as noted above. This places the FSM into the OUT state, where it waits for another HIGH signal on the same
 * HID pin. Once received, it enters MOVE_IN and both motors turn backwards until full retraction is determined.
 * At this point, the FSM moves back into the IN state, where it waits to repeat the process.
 * 
 * The Arduino also has three pins for LED output, which signal its current FSM state. Both static states (IN, OUT)
 * are signified by a static LED (binary 100 or 001), while both MOVE states are signified by rapid LED flashes in
 * a leftward or rightward drection (state-dependent). 
 * 
 * Outside of "typical" operation, the FSM also contains a user OVERRIDE state, which allows the user to move either 
 * motor according to the positioning of a potentiometer. This potentiometer must be calibrated; this code assumes
 * an OFF resistance of ____ and ON resistance of _____, with CENTER about ~______. When the potentiometer is 
 * centered, the FSM does not enter OVERRIDE, but if it is turned to either extreme, the corresponding motor will turn 
 * continuously backward or forward. This state is also signified by the LEDs going to an ALL ON state (binary 111).
 * 
 * Once the FSM has entered the OVERRIDE state, it can be "freed" back to normal operation by returning the
 * potentiometer to center and pressing the HID State Advance button. The FSM resumes operation in the MOVE_OUT 
 * state, to attempt full extension. To skip to a different state, the HID State Advance button may be pressed at
 * any point in the typical operation.
 * 
 * Notes: 
 *   Stepper used in testing is the 28BYJ-48 5V DC Motor. It has ~50 Ohms per phase. If using a more powerful motor,
 *   the per-phase resistance will likely be ~1-3 Ohms. Eg. Typical NEMA17 motors. DO NOT connect a 5V supply,
 *   as the Motorshield cannot current-limit. This will burn out the Motorshield, the motor, or both.
 *   
 *   You can safely power a low-impedance motor so long as the current is < 1.2A PER MOTOR. This is not the same
 *   as the per-phase current. For a bipolar stepper, per-motor current = 2 * (per-phase current). 
 *   
 *   Do not use the Arduino power source. Use an external DC power supply, hooked to the Motorshield 5-12V ports.
 *   Remove the VIN jumper **before** connecting the external power supply!
 *   
 *   Despite the above, because the motor will generate considerable back-EMF, even driving at Vin >> Vrated *might* 
 *   be ok. Consider using an external driver such as a "chopper" driver, which senses the output current and "chops"
 *   the voltage when necessary; alternatively, use a current-controlled power supply. (I have had success with a
 *   current-limited buck converter.)
 *   
 *   
 *   NS = Next State. PS = Present State.
 */


// Libraries
// Motors
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>

// Timer
#include <arduino-timer.h>
#include <elapsedMillis.h>

// ---------------------


// Timer Setup
Timer<10> timer; // 10 concurrent tasks, using millis as resolution

// ---------------------


// Motor Setup
const int stepsPerRevolution = 2048;  // Tells Stepper library how many steps = 1 revolution. 
                                      // (Needed for motor speed bc this sets RPMs.)

// Create MotorShield and Motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(stepsPerRevolution, 1); // M1 M2 stepper
Adafruit_StepperMotor *myStepper2 = AFMS.getStepper(stepsPerRevolution, 2); // M3 M4 stepper

// Motor Function Wrappers
// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
void forwardstep1() {  
  myStepper1->onestep(FORWARD, SINGLE);
}
void backwardstep1() {  
  myStepper1->onestep(BACKWARD, SINGLE);
}
void forwardstep2() {  
  myStepper2->onestep(FORWARD, DOUBLE);
}
void backwardstep2() {  
  myStepper2->onestep(BACKWARD, DOUBLE);
}

// Now we'll wrap the 2 steppers in an AccelStepper object.
// This will let us move the motors without blocking program flow.
// Note: When using a driver, requires a different constructor that specifies driver in use.
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);

// ----------------------------

// Globals

// States
enum OpState{IN, MOVE_OUT, OUT, MOVE_IN, OVERRIDE}; // Starts in "IN" state
OpState opState = IN; // Stores present state (PS)

// Limit Switches
bool limit1Pressed = false; // Limit Switches
bool limit2Pressed = false;
bool limit3Pressed = false;
bool limit4Pressed = false;
// Note that limit1&2 both read from limit1&2Pins. 
// Full implementation will require two additional switches. Could wire parallel to same pins?

// HID Controller
bool stateAdvancePressed = false;    // Button to move to next state.

// Boom Arms & Motor Targets
  // Motor stepsPerRevolution defined above.
#define boom1StepLength stepsPerRevolution*3;   // Number of motor steps for full extension of boom
#define boom2StepLength stepsPerRevolution*3;
long target1;                                   // These are how we'll tell motors 1&2 where to aim
long target2;
//Note: Set boom#StepLength values to large numbers if using limit switches at the ends of extension.
// If not using limit switches (eg. if the motor positioning accuracy is reliable), set it *smaller* or
// ideally **just at** the number of steps.
// For the motor I'm developing on, 2048 steps == 1 revolution, but other motors will be different.
// Check the datasheet to see the gearing and step size : degrees ratios.


// Pins
const int  limit1Pin = 2;    // Limit Switch 1
const int  limit2Pin = 3;    // Limit Switch 2
const int  limit3Pin = 2;     // Note: Change these to their own pins if you add limit switches 3&4!
const int  limit4Pin = 3;

const int  stateAdvancePin = 4;   // Tells program to move to next state. (Eg. To exit OUT / IN states.)

const int  opStateLED1 = 5;   // LED to signify motor movement states
const int  opStateLED2 = 6;
const int  opStateLED3 = 7;

// -------------------------

// Simple Debounce
// Reads from `pin`. If it has been < `debounce_time` since last read, it returns 0.
// Otherwise it will read the pin and return its value.
// Also returns a boolean `ifNewReading` which returns 1 if a reading was taken, 0 if not; this is regardless of the read value.
elapsedMillis timeElapsed = 1000;    // Auto-increments over time. Start at value >> debounce time.
int debounceMillisRead(int pin, long debounceMillis, bool &ifNewReading) {
  if (timeElapsed > debounceMillis) {    
    ifNewReading = true;      // Record that a reading was taken
    timeElapsed = 0;          // Reset timer for next debounce
    return(digitalRead(pin)); // Return the read value.
  }
  else {            // Return false, don't read.
    ifNewReading = false;     // Record that no reading was taken.
    return(0);                // Return false regardless of pin value.
  }
}

bool stateLogic() {
  
  // Query HID State Advance button
  bool dummyVar;
  stateAdvancePressed = debounceMillisRead(stateAdvancePin, 250, dummyVar);
  Serial.println(stateAdvancePressed);

  // NS Logic
  switch(opState) {
    
    case IN:          // Boom arms fully retracted. Start state.
      if(stateAdvancePressed) {
        
        /* STL: IN --> MOVE_OUT */
        // Reset NS flags
        limit1Pressed = 0;            // Reset limit switch detection
        limit2Pressed = 0;
        stateAdvancePressed = false;  // Reset state advance flag

        // State transition
        opState = MOVE_OUT;
      }
      break;

    /* STL: MOVE_OUT --> OUT */
    case MOVE_OUT:
      // NS Conditions: StateAdvance button pressed OR 
      //                both motors have finished moving.
      if( stateAdvancePressed ||
          (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) ) {
        //updateStepper();    // Must call this **before** changing state!
                            // Otherwise, stepper will continue in same direction for 1 cycle.

        // Reset NS flags
        stateAdvancePressed = false;

        // State transition
        opState = OUT;     
      }
      break;
    
    /* STL: OUT --> MOVE_IN */
    case OUT:
      if( stateAdvancePressed ) {
        // Reset NS flags
        stateAdvancePressed = false;
        limit3Pressed = 0;
        limit4Pressed = 0;

        // State Transition
        opState = MOVE_IN;
      }
      break;

    /* STL: MOVE_IN --> IN */
    case MOVE_IN:
      // NS Conditions: 
      if( stateAdvancePressed ||
          (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) ) {
        //updateStepper();  // Call this **first** before changing state!
                          // Motor needs to setSpeed(0)!

        // Reset NS flags
        stateAdvancePressed = false;

        // State Transition
        opState = IN;
      }
      break;
  }
  return(true);
}

bool checkLimits() {
  switch(opState) {
    
    case IN:
      // Do nothing.
      break;
      
    case MOVE_OUT:
      // Read limit switch values
      if(!limit1Pressed) {  // Limit switch 1
        limit1Pressed = digitalRead(limit1Pin); // Set limit1Pressed FLAG
      }
      if(!limit2Pressed) {  // Limit switch 2
        limit2Pressed = digitalRead(limit2Pin); // Set limit2Pressed FLAG
      }
      break;

    case OUT:
      // Do nothing.
      break;

    case MOVE_IN:
      // Check limit switches and read value
      if(!limit3Pressed) {  // Limit switch 3
        limit3Pressed = digitalRead(limit3Pin); // Set limit3Pressed FLAG
      }
      if(!limit4Pressed) {  // Limit switch 4
        limit4Pressed = digitalRead(limit4Pin); // Set limit4Pressed FLAG
      }
      break;
  }

  return(true); // Return value tells timer to keep looping or not. Keep returning TRUE.
}

void updateStepper() {
  switch(opState) {
    
    case IN:      // Boom arm retracted. Start state.
      // Set speed to 0.
      // If we got to this state by HID AdvanceState, the motor will still be programmed with its previous speed.
      stepper1.setSpeed(0);
      stepper2.setSpeed(0);
      
      stepper1.setCurrentPosition(0);   // Set START / HOME position as current position.
      stepper2.setCurrentPosition(0);
      break;
      
    case MOVE_OUT:   
      // Check if limit switch hit; if so, stop moving.      
      if (limit1Pressed) {
        stepper1.setSpeed(0);                   // Stop moving
        target1 = stepper1.currentPosition();   // Set target as current position
      } else {
        target1 = boom1StepLength;  // If not, then set the target to destination.
      }
      if (limit2Pressed) {
        stepper2.setSpeed(0);                  // Stop moving
        target2 = stepper2.currentPosition();  // Set target as current position
      } else {
        target2 = boom2StepLength;
      }

      // Set target movement & execute one motor "tick"
      stepper1.moveTo(target1);
      stepper2.moveTo(target2);
      stepper1.run();
      stepper2.run();
      break;

    case OUT:
      // Set speed to 0.
      // If we got to this state by HID AdvanceState, the motor will still be programmed with its previous speed.
      stepper1.setSpeed(0);
      stepper2.setSpeed(0);
      break;

    case MOVE_IN:  
      // Check if limit reached; if so, stop moving.
      if (limit3Pressed) {
        stepper1.setSpeed(0);                  // Stop moving
        target1 = stepper1.currentPosition();  // Set target as current position
                
        // Reset the limit switch
        //limit1Pressed = false;
      } else {
        target1 = 0;    // Same as before, if !limit then keep target destination as home.
      }
      if (limit4Pressed) {
        stepper2.setSpeed(0);                  // Stop moving
        target2 = stepper2.currentPosition();  // Set target as current position
    
        // Reset the limit switch
        //limit2Pressed = false;
      } else {
        target2 = 0;
      }

      // Set target movement & execute one motor "tick"
      stepper1.moveTo(target1);
      stepper2.moveTo(target2);
      stepper1.run(); 
      stepper2.run();     // Note: run() is what updates distanceToGo().
      break;
  }
}


// Controls LEDs to light up indicating the current state:
    // IN: First LED lit
    // MOVE_OUT: Cycle LEDs right
    // OUT: Last LED lit
    // MOVE_IN: Cycle LEDs left
    // OVERRIDE: All LEDs ON
bool opStateLEDBits[] = {1, 0, 0}; // "ON OFF OFF"
bool opStateIndicate() {
  bool temp;
  
  switch(opState) {
    case IN:
      digitalWrite(opStateLED1, HIGH);
      digitalWrite(opStateLED2, LOW);
      digitalWrite(opStateLED3, LOW);
      break;
      
    case MOVE_OUT:  // Read bits off to cycle LEDs --> 100,010,001,100,...  --> this way -->
      // Shift LED "bits"
      temp = opStateLEDBits[0];
      opStateLEDBits[0] = opStateLEDBits[2];
      opStateLEDBits[2] = opStateLEDBits[1];
      opStateLEDBits[1] = temp;

      // Write new output states
      digitalWrite(opStateLED1, opStateLEDBits[0]);
      digitalWrite(opStateLED2, opStateLEDBits[1]);
      digitalWrite(opStateLED3, opStateLEDBits[2]);
      
      break;
      
    case OUT:
      digitalWrite(opStateLED1, LOW);
      digitalWrite(opStateLED2, LOW);
      digitalWrite(opStateLED3, HIGH);
      break;
      
    case MOVE_IN: // Reads bits off to cycle LEDs --> 100,001,010,100,...  <-- this way <--
      // Shift LED "bits"
      temp = opStateLEDBits[0];
      opStateLEDBits[0] = opStateLEDBits[1];
      opStateLEDBits[1] = opStateLEDBits[2];
      opStateLEDBits[2] = temp;

      // Write new output states
      digitalWrite(opStateLED1, opStateLEDBits[0]);
      digitalWrite(opStateLED2, opStateLEDBits[1]);
      digitalWrite(opStateLED3, opStateLEDBits[2]);
      
      break;
      
    case OVERRIDE:
      digitalWrite(opStateLED1, HIGH);
      digitalWrite(opStateLED2, HIGH);
      digitalWrite(opStateLED3, HIGH);
      break;
  }
  return(true);
}

// -------------

void setup()
{  
  // Serial Monitor
  Serial.begin(9600); // Start Serial Monitor

  // Set Pins for Limit Switches
  pinMode(limit1Pin, INPUT);
  pinMode(limit2Pin, INPUT);
  pinMode(limit3Pin, INPUT);
  pinMode(limit4Pin, INPUT);

  // Set Pins for HID
  pinMode(stateAdvancePin, INPUT);  // State Advance button
  
  // Set Pins for LED
  pinMode(opStateLED1, OUTPUT);
  pinMode(opStateLED2, OUTPUT);
  pinMode(opStateLED3, OUTPUT);
  digitalWrite(opStateLED1, LOW);
  digitalWrite(opStateLED2, LOW);
  digitalWrite(opStateLED3, LOW);

  // Timer Setup
 // timer.every(100, checkLimit); // Limit Switch readings every __ ms
  timer.every(150, opStateIndicate);  // Update LED indicators

  // Motor Setup
  AFMS.begin(); // Start the bottom shield

  // Motor 1
  stepper1.setMaxSpeed(500.0);
  stepper1.setAcceleration(300.0);      // For testing: change this value lower in implementation.
  // Motor 2
  stepper2.setMaxSpeed(500.0);
  stepper2.setAcceleration(300.0);      // For testing: change this value lower in implementation.
}

void loop()
{
  timer.tick();
  checkLimits();      // Query limit switches
  updateStepper();    // Update Stepper targets & move
  // opStateIndicate();  // Flash LED indicators
  
  stateLogic();       // STL: State Transition Logic
  
}
