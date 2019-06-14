# Spin-Doctor---Meltybrain-Software

External libraries used:
Sparkfun LIS331
https://github.com/sparkfun/Triple_Axis_Accelerometer_Breakout-H3LIS331DL
LinearRegressino
https://github.com/cubiwan/LinearRegressino
CPPM-RX
https://github.com/daPhoosa/CPPM-RX
DShot routines from bladeBench, updated for Teensy pins
https://github.com/Extent421/bladeBench

Hardware used:
Teensy 3.2 microcontroller
SparkFun H3LIS331DL accelerometer
6 channel CPPM receiver (and transmitter)
DShot600 compatible ESC
750kv brushless motor

Usage:
The gear and flap switches are used to determine the operating mode, with a safe mode for signal loss or startup in active modes
Gear  Flap  Mode            Lights              Notes
____  ____  _______________ ___________________ _____
N/A   N/A   safe mode       Fast green flashing Signal loss or started in mode other than idle, motor off
off   off   idle mode       Solid green         Connected, motor off
off   on    melty mode      Solid red and green Green and red lights indicate reference and movement directions respectively
on    off   calibrate mode  Solid red           Lights differ in different calibrations
on    on    max spin mode   Solid red           Only available while spinning in melty mode

Safe mode:
Only entered before connecting and setting switches for idle mode and during signal loss.
motors set off.  Safe mode after powerup.

Idle mode:
Motors set off.  Safe mode after powerup.

Melty mode:
Throttle up to begin MeltyBrain drive.
Green light blinks for persistance of vision while spinning to indicate the heading of the robot.  The green lights are meant to be pointed away from the user as a point of reference, so the bot can then translate based on the receiver inputs.
Red light is used to indicate the direction given by the receiver inputs.

Rudder is used to rotate the reference light to point away from user.
Throttle controls throttle, scaled to the max speed readable by the accelerometer.
Aileron makes bot strafe left and right.
Elevator makes bot drive forward and backward.

Calibrate mode:
Starts in calibration select menu, blinking a number of times to indicate which calibration is selected.
Rudder is used to change calibration selection.
Elevator up to start selected calibration.
Return switches to idle mode positions with throttle up to save current calibration data.
  1 - Throttle calibration
    5 second red warning flash before starting motor, elevator down to cancel
    Use rudder to increase/decrease motor speed to minimum necessary for spinup
    Elevator down to save
    Automatic slow throttle up until accelerometer saturates, then stops spinning
  2 - Accelerometer Ccalibration
    5 second red warning flash before starting motor, elevator down to cancel
    Starts spinning at slow speed from throttle calibration
    Green light blinks at fixed interval
    Use rudder to adjust interval until light appears stationary
    Use elevator down to increase throttle to next level and start interval adjustment again
    After 20 interval adjustments, stops spinning and computes relationship between accelerometer and interval
    Returns to calibration selection with red light on when calculation is complete
  3 - Light direction calibration 
    5 second red warning flash before starting motor, elevator down to cancel
    Starts spinning at 2/3 max for better translation
    Green light shows where bot thinks it is pointing
    Elevator down to make the bot start/stop driving forward/backward in 1 sec increments
    Aileron left/right adjusts the direction of the bot driving forward/backward
    Rudder left/right to adjust the light direction to match the direction being driven
    Throttle high to finish    

Max spin mode:
Ignores the accelerometer and stick controls and sets the motor to maximum.  Intended to be activated in the middle of an arena to give maximum energy storage.
