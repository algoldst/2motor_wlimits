# 2 Motors w/ Limit Switches

## Final Program: AccelStepper_Motorshield
The full program runs two Stepper motors in coordination with the following:

- controls status LEDs
- reads two potentiometers as HID
- reads 4 limit switches
- reads 1 HID button for state control

## File Hierarchy:

- `Stepper_Speed_Control`: This is a basic sketch for running a Stepper motor.
- `Stepper_Motorshield`: Adapts the basic sketch for running a Stepper motor, using the Motorshield v2.3.
- `AccelStepper_Motorshield`: Full program that runs two Stepper motors using a Motorshield and AccelStepper library. 

## Wiring Diagram
<img src="Wiring Diagram/Wiring Diagram.png">
