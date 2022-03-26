// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivetrain.DriveByJoystickCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.RotateToHeadingCommand;

/** Robot for testing/calibrating rotate-to-heading
 * 
 *  [ ] Manually drive
 *  [ ] Tune parameters
 *  [ ] Hardcode result in RotateToHeadingCommand and configure timeout
 */
public class RotateToHeadingTestRobot extends TimedRobot
{
    private final TalonSRX pigeon_carrier = new TalonSRX(RobotMap.LEFT_INTAKE);
    private final Drivetrain drivetrain = new Drivetrain(pigeon_carrier);
    private final DriveByJoystickCommand joydrive = new DriveByJoystickCommand(drivetrain);
    private final RotateToHeadingCommand rotate = new RotateToHeadingCommand(drivetrain, 0.0);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 RotateToHeading Test *****");

        // Allow setting the speed used during auto-no-mouse
        SmartDashboard.setDefaultNumber("Desired Heading", 0.0);

        SmartDashboard.setDefaultNumber("Tolerance", 1.0);
        SmartDashboard.setDefaultNumber("P", 0.005); // 0.5 / 90, get half speed for 90 deg. error
        SmartDashboard.setDefaultNumber("I", 0.0);
        SmartDashboard.setDefaultNumber("D", 0.0);   // 0.05
        SmartDashboard.setDefaultNumber("max", 0.5);
    }

    @Override
    public void robotPeriodic()
    {
        // Make commands and subsystems execute
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit()
    {
        drivetrain.reset();
        joydrive.schedule();
    }

    @Override
    public void autonomousInit()
    {
        drivetrain.reset();
        rotate.schedule();
    }

    @Override
    public void autonomousPeriodic()
    {
        // Allow adjusting PID from dashboard
        rotate.configure(SmartDashboard.getNumber("Desired Heading", 0.0),
                         SmartDashboard.getNumber("Tolerance", 0.0),
                         SmartDashboard.getNumber("P", 0.0),
                         SmartDashboard.getNumber("I", 0.0),
                         SmartDashboard.getNumber("D", 0.0),
                         SmartDashboard.getNumber("max", 0.0));
    }
}
