/*
 * Author: SMRAZA KEEN
 * Date:2016/6/29
 * IDE V1.6.9
 * Email:TechnicSmraza@outlook.com
 * 
 * Modified by Alex Goldstein
 */
#include <Stepper.h>
const int stepsPerRevolution = 2048;  // Tells Stepper library how many steps = 1 revolution. 
                                      // (Needed for motor speed bc this sets RPMs.)

// Initialize a Stepper object on pins 2, 4, 3, 5.
// (Order matters — this tells Stepper what order to power the coils.)
Stepper myStepper(stepsPerRevolution, 2, 4, 3, 5);

int stepCount = 0;  // number of steps the motor has taken
void setup() {
  Serial.begin(9600);
}

void loop() {
  // If we had a sensor, we could read its value,
  // then map it to a range from 0 to 19 to set RPMs.
  //int sensorReading = analogRead(A0);
  //int motorSpeed = map(sensorReading, 0, 1023, 0, 19);

  // Set rotation speed
  int motorSpeed = 15;  // Controls RPMs. Testing --> my motor slips at > 20rpm w/no load.
                        // More likely to slip at higher RPMs.
  myStepper.setSpeed(motorSpeed); // RPMs

  // Rotate!
  myStepper.step(-stepsPerRevolution);  // Choose # steps to rotate.
                                        // 1 rev == stepsPerRevolution.
                                        // Negative values reverse direction.
                                        // Blocking function.
  Serial.print("Outputting");
  delay(300);                           // Delay proceeds after steps have completed.
}
