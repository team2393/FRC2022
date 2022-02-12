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
 * Test each component, have DriveTest, LiftTest, GadgetXYZTest .. robot code for each component to allow testing just that component
 * Add functional component software to overall robot code

March

 * Robot practice runs, improvements, tuning
 * Competitions


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

