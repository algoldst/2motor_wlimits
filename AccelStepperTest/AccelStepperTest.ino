
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
bool limit1Pressed = false; // Limit Switches
bool limit2Pressed = false;

// Pins
const int  limit1Pin = 2;    // Limit Switch 1
const int  limit2Pin = 3;    // Limit Switch 2

void setup()
{  
  // Serial Monitor
  Serial.begin(9600); // Start Serial Monitor

  // Set Pins for Button
  // initialize the button pin as a input:
  pinMode(limit1Pin, INPUT);
  pinMode(limit2Pin, INPUT);

  // Timer Setup
  timer.every(200, checkLimit);

  // Motor Setup
  AFMS.begin(); // Start the bottom shield

  // Motor 1
  stepper1.setMaxSpeed(500.0);
  stepper1.setAcceleration(50.0);
  stepper1.moveTo(stepsPerRevolution);
  // Motor 2
  stepper2.setMaxSpeed(200.0);
  stepper2.setAcceleration(100.0);
  stepper2.moveTo(50000);
}

void loop()
{
  updateStepper();
  timer.tick();
  
  if(limit1Pressed) {
    Serial.print("Gotpressed");
  }
}

bool checkLimit() {
  if(!limit1Pressed) {
    limit1Pressed = 0; //digitalRead(limit1Pin);
  }
  if(!limit2Pressed) {
    limit2Pressed = digitalRead(limit2Pin);
  }
  Serial.println(limit1Pressed);
  return(true);  
}

int target = -stepsPerRevolution;
void updateStepper() {
  // Change direction at the limits
  if (stepper1.distanceToGo() == 0 || limit1Pressed) {
    //stepper1.moveTo(-stepper1.currentPosition());
    //stepper1.moveTo(stepper1.currentPosition()); // Stop moving
    stepper1.setSpeed(0);                         // Stop moving
    delay(500);
    stepper1.moveTo(target);  // Set target as current position
    target = -target;

    // Reset the limit switch
    limit1Pressed = false;
  }
  if (stepper2.distanceToGo() == 0 || limit2Pressed) {
    stepper1.setSpeed(0);                         // Stop moving
    stepper1.moveTo(stepper1.currentPosition());  // Set target as current position

    // Reset the limit switch
    limit2Pressed = false;
  }
  
  // Movement "tick"
  stepper1.run();
  stepper2.run();
}
