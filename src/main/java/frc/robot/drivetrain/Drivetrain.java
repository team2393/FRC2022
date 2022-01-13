// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Motors, encoders, .. of the drive chassis */
public class Drivetrain extends SubsystemBase
{
    private final WPI_TalonFX primary_left = new WPI_TalonFX(RobotMap.PRIMARY_LEFT_DRIVE);
    private final WPI_TalonFX secondary_left = new WPI_TalonFX(RobotMap.SECONDARY_LEFT_DRIVE);
    private final WPI_TalonFX primary_right = new WPI_TalonFX(RobotMap.PRIMARY_RIGHT_DRIVE);
    private final WPI_TalonFX secondary_right = new WPI_TalonFX(RobotMap.SECONDARY_RIGHT_DRIVE);

    // Combine into DifferentialDrive
    private final DifferentialDrive diff_drive = new DifferentialDrive(primary_left, primary_right);

    public Drivetrain()
    {
        // Motors on right need to be inverted
        initializeMotor(primary_left, false);
        initializeMotor(secondary_left, false);
        initializeMotor(primary_right, true);
        initializeMotor(secondary_right, true);

        // Have secondaries follow the primaries
        secondary_left.follow(primary_left);
        secondary_right.follow(primary_right);
    }

    /** @param motor Motor to initialize
     *  @param invert Should motor direction be inverted?
     */
    private void initializeMotor(WPI_TalonFX motor, boolean invert)
    {
        // Motors remember certain settings. We don't know if the motor
        // is fresh out of the box or had been used on a different robot.
        // ==> Make sure that we start with the default configuration.
        motor.configFactoryDefault();
        motor.clearStickyFaults();
        motor.setInverted(invert);
    }

    public void drive(double speed, double rotation)
    {
            diff_drive.arcadeDrive(speed, rotation);
    }

    // TODO: DriveByJoystickCommand

    /** Create a command that runs the drve train along a trajectory
     * 
     *  Trajectory starts at X=0, Y=0 and Heading = 0.
     *  Given list of points must contain entries x, y, h,
     *  i.e., total length of x_y_h array must be a multiple of 3.
     * 
     *  @param x_y_z Sequence of points { X, Y, Heading }
     */
    public CommandBase createTrajectoryCommand(double... x_y_h)
    {
        if (x_y_h.length % 3 != 0)
            throw new IllegalArgumentException("List of { X, Y, Heading } contains " + x_y_h.length + " entries?!");
        
        // TODO Create trajectory, return RamseteCommand to follow,
        //      instead of just printing what it should do
        final SequentialCommandGroup result = new SequentialCommandGroup();
        result.addCommands(new PrintCommand("Start at 0, 0, 0"));
        for (int i=0; i<x_y_h.length; i += 3)
            result.addCommands(new PrintCommand("Move to " + x_y_h[i] + ", " + x_y_h[i+1] + ", " + x_y_h[i+2]));
        result.addCommands(new PrintCommand("Stop"));

        result.addRequirements(this);

        return result;
    }
}
