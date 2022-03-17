// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.camera.RotateToTargetCommand;
import frc.robot.camera.SetSpeedForTargetCommand;
import frc.robot.drivetrain.DriveByJoystickCommand;
import frc.robot.drivetrain.Drivetrain;

/** Robot for camera tests
 * 
 *  Check with small 'rmin' if robot rotates the correct way
 *  Then tune 'rmin', 'P', 'rmax'.
 *  Check if 'SpinnerSetpoint' updates with distance
 */
public class CameraTestRobot extends TimedRobot
{
    private final WPI_TalonSRX pigeon_carrier = new WPI_TalonSRX(RobotMap.LEFT_INTAKE);
    private final Drivetrain drivetrain = new Drivetrain(pigeon_carrier);
    private final CommandBase joydrive = new DriveByJoystickCommand(drivetrain);
    private final RotateToTargetCommand rotate_to_target = new RotateToTargetCommand(drivetrain);
    private final CommandBase set_speed_command = new SetSpeedForTargetCommand("limelight-front");

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Camera Test *****");

        SmartDashboard.setDefaultNumber("rmin", 0.0);
        SmartDashboard.setDefaultNumber("P", 0.0);
        SmartDashboard.setDefaultNumber("rmax", 0.0);
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        // Faster updates
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void disabledInit()
    {
        joydrive.cancel();
    }

    @Override
    public void teleopInit()
    {
        joydrive.schedule();
        set_speed_command.schedule();
    }

    @Override
    public void autonomousInit()
    {
        rotate_to_target.schedule();
        set_speed_command.schedule();
    }

    @Override
    public void autonomousPeriodic()
    {
        rotate_to_target.configure(SmartDashboard.getNumber("rmin", 0),
                                   SmartDashboard.getNumber("P", 0),
                                   SmartDashboard.getNumber("rmax", 0));
    }
}
