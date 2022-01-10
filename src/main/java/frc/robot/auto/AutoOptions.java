// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drivetrain.Drivetrain;

/** Helper for creating auto options */
public class AutoOptions
{
    /** Create all the auto options
     *  @param auto_options Chooser for dashboard that will allow selecting an option
     *  @param drivetrain Drivetrain to use for moves
     */
    public static void populate(SendableChooser<CommandBase> auto_options,
                                final Drivetrain drivetrain)
    {
        // First list the default option that does nothing
        auto_options.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));

        auto_options.addOption("Hello", new PrintCommand("Hello!"));

        auto_options.addOption("DelayedBack",
           new SequentialCommandGroup(
               new PrintCommand("Back"),
               new WaitCommand(2.0),
               drivetrain.createTrajectoryCommand(-1.0, 0, 0)
           ));

        auto_options.addOption("ForwardAndBack",
           new SequentialCommandGroup(
               new PrintCommand("ForwardAndBack"),
               drivetrain.createTrajectoryCommand(1, 0, 0,
                                                  0, 0, 0)
           ));

    }
}