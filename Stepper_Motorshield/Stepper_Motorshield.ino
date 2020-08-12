/*
 * Author: SMRAZA KEEN
 * Date:2016/6/29
 * IDE V1.6.9
 * Email:TechnicSmraza@outlook.com
 * 
 * Modified by Alex Goldstein
 */

// Libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>

// Motor Constants
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

// Now we'll wrap the 3 steppers in an AccelStepper object
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);

void setup()
{  
  Serial.begin(9600); // Start Serial Monitor

  // Motor Setup
  AFMS.begin(); // Start the bottom shield

  // Motor 1
  stepper1.setMaxSpeed(100.0);
  stepper1.setAcceleration(100.0);
  stepper1.moveTo(24);
    
  stepper2.setMaxSpeed(200.0);
  stepper2.setAcceleration(100.0);
  stepper2.moveTo(50000);
}

void loop()
{
  // Change direction at the limits
  if (stepper1.distanceToGo() == 0)
    stepper1.moveTo(-stepper1.currentPosition());

  if (stepper2.distanceToGo() == 0)
    stepper2.moveTo(-stepper2.currentPosition());

  // Begin/continue movement
  stepper1.run();
  stepper2.run();

  Serial.println(stepper1.speed());
}
