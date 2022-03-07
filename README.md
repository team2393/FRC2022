FRC2022
=======

Code for 2022 Robot

 * Game manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system
 * Software
   * Basics of Java: https://www.codecademy.com/learn/learn-java, [Java Book](https://www.amazon.com/dp/0596009208),
     [Free Java Book](https://greenteapress.com/wp/think-java-2e/)
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

If you're later asked to commit changes, you'll also need to
  1) Get a github account
  2) Get added to the people with write access to the repository
  3) Create an "access token", https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token

If git doesn't remember the last used credentials:  `git config credential.helper manager`

Basic Plan and Progress
-----------------------

January 2022

* January 8: Get latest software .. Note that wpilib has breaking changes, so need older setup for TShirt robot. The radio setup also requires the older software, https://docs.wpilib.org/en/latest/docs/yearly-overview/known-issues.html
* Create skeleton robot code that does "nothing" but builds, uploads to RoboRIO, connects to drive station
* Learn about motors, encoders, feedforward for 'SimpleMotor', 'Elevator', 'Arm'
* PID for speed control, PID for position control
* Drivetrain: Basic teleop, calibrate encoders, speed control
* Spinner: Basic teleop, speed control
* The Analog Devices gyro that we used last year only reports 0 degrees, never updating.
  Looks like known issue https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html?highlight=gyro (avoid by using pigeon)
* Color sensor might be usable to detect balls and their color, but needs to be plugged into MXP, see known issue
  on on-board I2C lockup
* Test new REV power distribution and pneumatics hub
* Drivetrain: Add gyro (pigeon), odometry, trajectory generation, autonomous drive
* Vision processing on raspberry pi: Detect blue vs. red cargo. Test with servo. Add DriveAndRotateToVisionCommand.
* Start skeleton software for any expected robot component: Ball pickup, ball feeder, shooter
* Select high/low ejection angle
* Start skeleton for climber/arm
* Added light to camera

February

 * As robot is being built, have software ready for all components
 * Test each component, add functional software for each subsystem to overall robot code
   * DriveTrainTestRobot: After deploying, use Phoenix tuner to locate all motors.
     Basic drive test.
   * ManualIntakeTestRobot: Open/close, pull in ball
   * PneumaticHubTestRobot: Does compressor turn on? Check pressure indication
   * SpinnerTestRobot: Speed
   * ManualBallHandling: Move ball, eject, detect ejection, determine location for sensors
   * BallHandlingTestRobot: Take balls in automatically, spin up to eject, eject, keep spinner on for 2 secs, ..., reverse to de-clog
   * ArmTestRobot: Move arm, test limit switch, calibrate extension length, tune PID
   * BallHandlingTestRobot: Test ball eject detection
  
March

 * Robot practice runs, improvements, tuning
   * Fix IP of roboRIO
   * Test ClimbSequence, find button/switch assignments
   * Test AsynchronousInterrupt for climber limit switches
   * SysId for drivetrain
   * Commands on dashboard to configure 'LowGoalClose', 'LowGoalDist', 'HighGoal', ...
   * Commands for ball handling: Open, close, eject
   * Enable AutoShiftCommand, test it
   * Create auto sequences for competition
   * Set limelight address to 11 (front), 12 (back)
   * Test drivertrain brake off in disable
   * Test updated climber 'home' and climb sequence
   * Test new 'reset'
   * BallHandlingTestRobot:
     Plot "Spinner RPS", "Spinner Current Change", "SpinnerState", "Ball Ejected", then shoot balls in teleop.
     Set BallHandling.at_speed_filter to 2 seconds, check that we stay in SPINUP for that long after reaching speed.
     Revert to 10 cycles.
     Set Spinner.delay to 2 seconds, check that return to IDLE takes that long. Revert to small setting.
     Set Spinner.remember_shot to ~0.02 and check if it still works fine.
     Auto-shoot 2 balls, adjust Spinner.remember_shot to find point where 2nd ball fails, back off.

   * Faster ActiveArm?
   * Front camera: Calibrate targeting, test RotateToTargetCommand
   * Back camera: Calibrate ball detection, create/test RotateToBallCommand
 * Competitions


Radio
-----

Connect power to network hub.
Plug USB/Ethernet adapter into laptop and hub.
Connect radio to hub, but not, yet, power.
Right-click wifi indicator in toolbar, "Open Network & Internet Settings", "Change adapter options",
disable all but the USB/Ethernet adapter.
Start FRC radio configuration tool as administrator.
Configure team 2393, "Load Firmware", connect power to radio.
On success, enter "Robot Name" and press "Configure".


Limelight
---------

Initially, access as http://limelight.local:5801/

Each camera should have fixed IP in the range 10.23.93.11, ..12, .. to not conflict with FMS

"Front" Camera:
Set team 2393.
Set IP address to static, 10.23.93.11, allowing access as http://10.23.93.11:5801/, and change name to "limelight-front".
Example pipelines are in limelight folder. Load "Drive" to front camera.
Pipeline 0: DriveFront
Pipeline 1: Target (and make that the default)

"Back" Camera:
Set team 2393.
Set IP address to static, 10.23.93.12, allowing access as http://10.23.93.12:5801/, and change name to "limelight-back".
Pipeline 0: DriveBack (and make that the default)


Raspberry Pi
------------

http://10.29.93.36, see also FRC2022Pi repo


Profiling
---------

'VisualVM', available from https://visualvm.github.io,
allows you to see how much CPU and memory the code is using on the RoboRIO,
and where it spends its time.

In build.gradle, this addition to the FRCJavaArtifact section
allows VisualVM to access the JVM running on the robot:

```
frcJava(getArtifactTypeClass('FRCJavaArtifact')) {
    jvmArgs.add("-Dcom.sun.management.jmxremote=true")
    jvmArgs.add("-Dcom.sun.management.jmxremote.port=1099")
    jvmArgs.add("-Dcom.sun.management.jmxremote.local.only=false")
    jvmArgs.add("-Dcom.sun.management.jmxremote.ssl=false")
    jvmArgs.add("-Dcom.sun.management.jmxremote.authenticate=false")
    jvmArgs.add("-Djava.rmi.server.hostname=10.23.93.2")
}
```

Assuming you unpacked it to \Users\Public\wpilib,
you may have to start it from a command prompt to pass the JDK location:

```
cd \Users\Public\wpilib\visualvm\bin
visualvm --jdkhome \Users\Public\wpilib\2022\jdk
```

To connect to the program running on the robot:
 * File, Add JMX Connection
 * 'Connection:' 172.22.11.2:1099 respectively 10.23.93.2:1099
   (may uwse 1199 because of conflict with CAN supports)
 * Check 'Do not require SSL connection'
 * A new entry with a 'pid' should appear under the 'Remote' list.
   Double-click, then check 'Monitor', 'Sample.. CPU' etc.

