FRC2022
=======

Code for 2022 Robot

TODO
----

ASAP

 * Preview what's new in 2022 wpilib, https://docs.wpilib.org/en/latest/
 * Other resources: https://www.chiefdelphi.com/, https://www.andymark.com/
 * Do we need a new RoboRIO? Does it have to be the new version 2.0, https://www.andymark.com/products/ni-roborio-2, https://www.firstinspires.org/robotics/frc/blog/2021-ni-guest-blog-roborio2-0

January 2022

* January 8, get latest software .. Note that wpilib has breaking changes, so need older setup for TShirt robot. The radio setup also requires the older software, https://docs.wpilib.org/en/latest/docs/yearly-overview/known-issues.html
* Create skeleton robot code that does "nothing" but builds, uploads to RoboRIO, connects to drive station
* See what's new in hardware. Do we need to order new controller, radio? Are there new motor controllers, motors, sensors, .. that we need?
* Learn about motors, encoders
* PID for position control
* PID for speed control
* Feedforward for 'SimpleMotor', 'Elevator', 'Arm'
* Odometry, trajectory generation, autonomous drive with 'smashbot'
* Vision processing on raspberry pi
* Start skeleton software for any expected robot component: Lift? Arm? Gadget that detects color?

February

 * As robot is being built, have software ready for all components
 * Test each component, have DriveTest, LiftTest, GadgetXYZTest .. robot code for each component to allow testing just that component
 * Add functional component software to overall robot code

March

 * Robot practice runs, improvements, tuning
 * Competitions
