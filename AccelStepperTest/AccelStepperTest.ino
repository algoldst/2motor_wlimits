
// Libraries
// Motors
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>

// Timer
#include <arduino-timer.h>

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

// Now we'll wrap the 3 steppers in an AccelStepper object.
// This will let us move the motors without blocking program flow.
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);

// ----------------------------

// Globals

// States
enum OpState{IN, MOVE_OUT, OUT, MOVE_IN, OVERRIDE}; // Starts in "IN" state
OpState opState = IN;

int doNothing = 0;  // Otherwise the compiler deletes parts of the code as "cleanup".

// Limit Switches
bool limit1Pressed = false; // Limit Switches
bool limit2Pressed = false;
bool limit3Pressed = false;
bool limit4Pressed = false;
// Note that limit1&2 both read from limit1&2Pins. 
// Full implementation will require two additional switches. Could wire parallel to same pins?

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

const int  opStateLED1 = 5;   // LED to signify motor movement states
const int  opStateLED2 = 6;
const int  opStateLED3 = 7;

// -------------------------


bool checkLimit() {
  switch(opState) {
    
    case IN:          // Boom arms fully retracted. Start state.
      
      if(millis() > 3000) {  // Wait 3sec before deploying.
        // IN --> MOVE_OUT  State transition logic
        opState = MOVE_OUT;
        
        // Set motor 1&2 travel target
        target1 = boom1StepLength;
        target2 = boom2StepLength;
      }
      break;
      
    case MOVE_OUT:    // Boom arms deploying out

      // Check limit switches and read value
      if(!limit1Pressed) {  // Limit switch 1
        limit1Pressed = digitalRead(limit1Pin); // Set limit1Pressed FLAG
      } else { 
        Serial.print("Limit1"); // Serial Monitor update when detected.
      }
      if(!limit2Pressed) {  // Limit switch 2
        limit2Pressed = digitalRead(limit2Pin); // Set limit2Pressed FLAG
      } else {
        Serial.print("Limit2"); // Serial Monitor update when detected.
      }

      // MOVE_OUT --> OUT
      // State change / transition logic if both booms deployed
      if(limit1Pressed && limit2Pressed) {
        updateStepper();  // Call this **first** before changing state!
        
        opState = OUT;

        // Reset Limit Switches (not used any more; we're only retracting afterward)
        limit1Pressed = 0;
        limit2Pressed = 0;
      }
      break;

    case OUT:       // Boom arms fully deployed.
      // Do nothing.
      delay(1000); // Blocking delay.

      // OUT --> MOVE_IN
      // State change / transition logic
      opState = MOVE_IN;

      target1 = 0;
      target2 = 0;
      
      break;

    case MOVE_IN:    // Boom arms retracting in

      // Check limit switches and read value
      if(!limit3Pressed) {  // Limit switch 3
        limit3Pressed = digitalRead(limit1Pin); // Set limit3Pressed FLAG
      } else { 
        Serial.print("Limit3"); // Serial Monitor update when detected.
      }
      if(!limit4Pressed) {  // Limit switch 4
        limit4Pressed = digitalRead(limit2Pin); // Set limit4Pressed FLAG
      } else {
        Serial.print("Limit4"); // Serial Monitor update when detected.
      }

      // MOVE_IN --> IN
      // State change logic if both booms retracted
      if(limit3Pressed && limit4Pressed) {
        updateStepper();  // Call this **first** before changing state!
                          // Motor needs to setSpeed(0)!
        opState = IN;

        // Reset Limit Switches (not used any more; we're only extending afterward)
        limit3Pressed = 0;
        limit4Pressed = 0;
      }
      break;
  }

  return(true); // Return value tells timer to keep looping or not. Keep returning TRUE.
}

void updateStepper() {
  switch(opState) {
    
    case IN:      // Boom arm retracted. Start state.
      stepper1.setCurrentPosition(0);   // Set START / HOME position as current position.
      stepper2.setCurrentPosition(0);
      break;
      
    case MOVE_OUT:   
      // Check if limit switch hit; if not, move to target.      
      if (limit1Pressed) {
        stepper1.setSpeed(0);                         // Stop moving
        target1 = stepper1.currentPosition();  // Set target as current position
                
        // Reset the limit switch
        //limit1Pressed = false;
      }
      if (limit2Pressed) {
        stepper2.setSpeed(0);                         // Stop moving
        target2 = stepper2.currentPosition();  // Set target as current position
    
        // Reset the limit switch
        //limit2Pressed = false;
      }

      // Set target movement & execute one motor "tick"
      stepper1.moveTo(target1);
      stepper2.moveTo(target2);
      stepper1.run();
      stepper2.run();
      break;

    case OUT:
      // Do nothing.
      break;

    case MOVE_IN:  
      // Check if limit reached; if not, move to home.
      if (limit3Pressed) {
        stepper1.setSpeed(0);                         // Stop moving
        target1 = stepper1.currentPosition();  // Set target as current position
                
        // Reset the limit switch
        //limit1Pressed = false;
      } 
      if (limit4Pressed) {
        stepper2.setSpeed(0);                         // Stop moving
        target2 = stepper2.currentPosition();  // Set target as current position
    
        // Reset the limit switch
        //limit2Pressed = false;
      }

      // Set target movement & execute one motor "tick"
      stepper1.moveTo(target1);
      stepper2.moveTo(target2);
      stepper1.run(); 
      stepper2.run();
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

void opStateIndicate() {
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
      
//      Serial.print(opStateLEDBits[0]);
//      Serial.print(opStateLEDBits[1]);
//      Serial.println(opStateLEDBits[2]);
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
      
//      Serial.print(opStateLEDBits[0]);
//      Serial.print(opStateLEDBits[1]);
//      Serial.println(opStateLEDBits[2]);
      break;
      
    case OVERRIDE:
      digitalWrite(opStateLED1, HIGH);
      digitalWrite(opStateLED2, HIGH);
      digitalWrite(opStateLED3, HIGH);
      break;
  }
  
}

// -------------

void setup()
{  
  // Serial Monitor
  Serial.begin(9600); // Start Serial Monitor

  // Set Pins for Button
  // initialize the button pin as a input:
  pinMode(limit1Pin, INPUT);
  pinMode(limit2Pin, INPUT);

  // Set Pins for LED
  pinMode(opStateLED1, OUTPUT);
  pinMode(opStateLED2, OUTPUT);
  pinMode(opStateLED3, OUTPUT);
  digitalWrite(opStateLED1, LOW);
  digitalWrite(opStateLED2, LOW);
  digitalWrite(opStateLED3, LOW);

  // Timer Setup
  timer.every(100, checkLimit); // Limit Switch readings every __ ms
  timer.every(120, opStateIndicate);  // Update LED indicators

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
  updateStepper();
}
