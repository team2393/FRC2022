     // Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.cargo.BallHandling;
import frc.robot.cargo.CloseIntakeCommand;
import frc.robot.cargo.OpenIntakeCommand;
import frc.robot.cargo.ShootCommand;
import frc.robot.cargo.ToggleSpinnerCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.ShiftLowCommand;
import frc.robot.drivetrain.StayPutCommand;

/** Helper for creating auto options */
public class AutoOptions
{
    /** Create all the auto options
     *  @param auto_options Chooser for dashboard that will allow selecting an option
     *  @param drivetrain Drivetrain to use for moves
     */
    public static void populate(SendableChooser<CommandBase> auto_options,
                                final Drivetrain drivetrain,
                                final BallHandling ball_handling)
    {
        // First list the default option that does nothing
        auto_options.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));

        {   // Drive 1 m backwards
            auto_options.addOption("TAXI (A1FF)",
                new SequentialCommandGroup(
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(false, -1, 0, 0)));
        }

        {  // Drive 1 m backwards after 10 seconds
            auto_options.addOption("TAXIDelayed (A1FF)",
            new SequentialCommandGroup(
                new ShiftLowCommand(drivetrain),
                new WaitCommand(10.0),
                drivetrain.createTrajectoryCommand(false, -1, 0, 0)));
        }

        {   // Left Tarmac, Middle Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.26, 0.003, -1);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(2.167, 0.0487, 24.3));
    
            auto_options.addOption("LTMEPickShootShoot (CTFDU)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", false),
                    new ApplySettingCommand("SpinnerSetpoint", 61),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),                   
                    drivetrain.createTrajectoryCommand(seg1),
                    drivetrain.createTrajectoryCommand(seg2),
                    new ShootCommand(ball_handling),
                    new WaitCommand(1.5),
                    new ShootCommand(ball_handling)));
        }

        {   // Left Tarmac, Right Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.335, -1.27, 42);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(2.3, 0.05, -26,
                                                  2.825, -0.312, -80));
    
            auto_options.addOption("LTREPickShootShoot (Hamburger)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", false),
                    new ApplySettingCommand("SpinnerSetpoint", 61),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),                   
                    drivetrain.createTrajectoryCommand(seg1),
                    drivetrain.createTrajectoryCommand(seg2),
                    new ShootCommand(ball_handling),
                    new WaitCommand(1.5),
                    new ShootCommand(ball_handling)));
        }

        {   // Left Tarmac, Left Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.239, 0.0, 0);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(2.376, -0.407, 48));
    
            auto_options.addOption("LTLETaxiShoot (GG)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", false),
                    new ApplySettingCommand("SpinnerSetpoint", 61),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(seg1),
                    drivetrain.createTrajectoryCommand(seg2),
                    new ShootCommand(ball_handling)));
        }

        {   // Right Tarmac, Left Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false, -1.29, -0.12, 14);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(2.2, -0.8, 26));
    
            auto_options.addOption("RTLEPickShootShoot (SM)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", false),
                    new ApplySettingCommand("SpinnerSetpoint", 61),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),          
                    drivetrain.createTrajectoryCommand(seg1),
                    drivetrain.createTrajectoryCommand(seg2),
                    new ShootCommand(ball_handling),
                    new WaitCommand(1.5),
                    new ShootCommand(ball_handling)));
        }

        // --------------- Non-competition test moves ----------------------------
        {   // ~1m forward and right
            Trajectory seg1 = TrajectoryHelper.createTrajectory(1.3, -0.94, -90);
            // ..and from there on ~1m back and left
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                                    TrajectoryHelper.createTrajectory(false, -1.3, 0.9, -90));
            auto_options.addOption("Test1",
                new SequentialCommandGroup(
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(seg1),
                    // See also ParallelRaceGroup(), ParallelDeadlineGroup()
                    // Use with StayPutCommand() to avoid motor timeouts!
                    new ParallelDeadlineGroup(
                        new ShootCommand(ball_handling),
                        new StayPutCommand(drivetrain)
                    ),
                    new ParallelCommandGroup(
                        new OpenIntakeCommand(ball_handling),
                        drivetrain.createTrajectoryCommand(seg2)
                    ),
                    new CloseIntakeCommand(ball_handling),
                    new ParallelDeadlineGroup(
                        new ShootCommand(ball_handling),
                        new StayPutCommand(drivetrain)
                    )
                ));

            auto_options.addOption("Round",
                new SequentialCommandGroup(
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(3, 0, 45,
                                                       3, 1, 135,
                                                       0, 1, 225,
                                                       0, 0, 0)));
        }
    }
}