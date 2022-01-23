FRC2022
=======

Code for 2022 Robot

 * Game manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system
 * Software
   * Basics of Java: https://www.codecademy.com/learn/learn-java, [Java Book](https://www.amazon.com/dp/0596009208)
   * Robot "WPILib", the FRC software framework https://docs.wpilib.org/en/stable/
   * Past [getting-started sessions](https://github.com/team2393/FRC/wiki), [code examples](https://github.com/team2393/FRC/tree/master/src/main/java)
   * Cross-the-Road Electronics, Phoenix tuner and manuals for several CAN devices: https://newsite.ctr-electronics.com/
   * REV, new power distribution and pneumatics hub: https://docs.revrobotics.com/docs/first-robotics-competition
   * Other resources: https://www.chiefdelphi.com/, https://www.andymark.com/, https://docs.photonvision.org

To get a copy of the sources, follow the WPILib introduction section on "Installing Software".
You mostly need the "WPILib", which includes VS Code, and can add the rest later.
Then get GIT from https://git-scm.com/downloads .
Finally, open VS Code, and select the View menu, Command Palette, Git: Clone, `https://github.com/team2393/FRC2022.git` .
When prompted for the location of the code, best use a `git/` subfolder of your home directory.

If you're asked to commit changes, you'll also need to
  1) Get a github account
  2) Get added to the people with write access to the repository
  3) Create an "access token", https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token


Basic Plan and Progress
-----------------------

January 2022

* January 8: Get latest software .. Note that wpilib has breaking changes, so need older setup for TShirt robot. The radio setup also requires the older software, https://docs.wpilib.org/en/latest/docs/yearly-overview/known-issues.html
* Create skeleton robot code that does "nothing" but builds, uploads to RoboRIO, connects to drive station
* Learn about motors, encoders, feedforward for 'SimpleMotor', 'Elevator', 'Arm'
* PID for speed control, PID for position control
* Drivetrain: Basic teleop, calibrate encoders, speed control
* Spinner: Basic teleop, speed control

Ran into two issues:
* In spinner teleop test, which works fine on another laptop, the robot room laptop stops reacting to the joystick.
  Looks like ongoing issue https://github.com/wpilibsuite/allwpilib/issues/3896 (resolved)
* The Analog Devices gyro that we used last year only reports 0 degrees, never updating.
  Looks like known issue https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html?highlight=gyro (avoid by using pigeon)
  
* Test new REV power distribution and pneumatics hub
* Drivetrain: Add gyro (pigeon), odometry, trajectory generation, autonomous drive
* Vision processing on raspberry pi: Detect blue vs. red cargo. Test with servo. Add DriveAndRotateToVisionCommand.
* Start skeleton software for any expected robot component: Ball pickup, ball feeder, shooter
* Select high/low ejection angle
* Start skeleton for climber/arm

Next:
* Add light to camera

February

 * As robot is being built, have software ready for all components
 * Test each component, have DriveTest, LiftTest, GadgetXYZTest .. robot code for each component to allow testing just that component
 * Add functional component software to overall robot code

March

 * Robot practice runs, improvements, tuning
 * Competitions
