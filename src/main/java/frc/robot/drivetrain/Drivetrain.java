// Copyright (c) Team 2993, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;

/** Motors, encoders, .. of the drive chassis */
public class Drivetrain
{
    private final WPI_TalonFX primary_left = new WPI_TalonFX(RobotMap.PRIMARY_LEFT_DRIVE);
    // TODO: All motors...

    // TODO: Combine into   DifferentialDrive



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
        return result;
    }
}
