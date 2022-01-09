FRC2022
=======

Code for 2022 Robot

 * Game manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system
 * Software: WPILIB https://docs.wpilib.org/en/stable/, CTRE https://newsite.ctr-electronics.com/, GIT https://git-scm.com/downloads
 * To get a copy of the sources into VS Code, use View, Command Palette, Git: Clone, https://github.com/team2393/FRC2022.git .
   Place it in a git/ subfolder of your home directory.
 * To commit, you need to
    1) A github account
    2) Ask to be added to the people with write access to the repository
    3) Create an "access token", https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token


TODO
----

ASAP

 * Preview what's new in 2022 wpilib, https://docs.wpilib.org/en/stable/, CTRE, https://newsite.ctr-electronics.com/
 * Other resources: https://www.chiefdelphi.com/, https://www.andymark.com/
 * Do we need a new RoboRIO? Does it have to be the new version 2.0, https://www.andymark.com/products/ni-roborio-2, https://www.firstinspires.org/robotics/frc/blog/2021-ni-guest-blog-roborio2-0

January 2022

* January 8: Get latest software .. Note that wpilib has breaking changes, so need older setup for TShirt robot. The radio setup also requires the older software, https://docs.wpilib.org/en/latest/docs/yearly-overview/known-issues.html
* Create skeleton robot code that does "nothing" but builds, uploads to RoboRIO, connects to drive station
* See what's new in hardware. Do we need to order new controller, radio? Are there new motor controllers, motors, sensors, .. that we need? https://www.andymark.com/products/climber-in-a-box, https://wcproducts.com/collections/viewall/products/greyt-shooter-9-5
* Learn about motors, encoders
* Feedforward for 'SimpleMotor', 'Elevator', 'Arm'
* PID for speed control
* PID for position control
* Odometry, trajectory generation, autonomous drive with 'smashbot'
* Vision processing on raspberry pi
* Start skeleton software for any expected robot component: Ball pickup, ball ejection, climber/arm, ...

February

 * As robot is being built, have software ready for all components
 * Test each component, have DriveTest, LiftTest, GadgetXYZTest .. robot code for each component to allow testing just that component
 * Add functional component software to overall robot code

March

 * Robot practice runs, improvements, tuning
 * Competitions
