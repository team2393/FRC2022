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

        {   // TODO test Drive 1 m backwards
            auto_options.addOption("TAXI",
                new SequentialCommandGroup(
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(false, -1, 0, 0)));
        }

        {  // TODO test Drive 1 m backwards after 10 seconds
            auto_options.addOption("TAXIDelayed",
            new SequentialCommandGroup(
                new ShiftLowCommand(drivetrain),
                new WaitCommand(10.0),
                drivetrain.createTrajectoryCommand(false, -1, 0, 0)));
        }

        {   // Shoot, pickup, shoot from middle of left area
            Trajectory seg1 = TrajectoryHelper.createTrajectory(1.045, -0.03, -18);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(false, -2.16, 0.73, -18));
            Trajectory seg3 = TrajectoryHelper.continueTrajectory(seg2,
                TrajectoryHelper.createTrajectory(2.24, 0.11, 19));
    
            auto_options.addOption("MidShootPickShoot",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", false),
                    new ApplySettingCommand("SpinnerSetpoint", 65),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(seg1),
                    new ParallelDeadlineGroup(
                        new ShootCommand(ball_handling),
                        new StayPutCommand(drivetrain)
                    ),
                    new OpenIntakeCommand(ball_handling),                   
                    drivetrain.createTrajectoryCommand(seg2),
                    drivetrain.createTrajectoryCommand(seg3),
                    new ParallelDeadlineGroup(
                        new ShootCommand(ball_handling),
                        new StayPutCommand(drivetrain)
                    )));
        }

        {   // TODO waypoints Left Tarmac, Middle Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(1.045, -0.03, -18);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(2.24, 0.11, 19));
    
            auto_options.addOption("LTMEPickShootShoot",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", false),
                    new ApplySettingCommand("SpinnerSetpoint", 65),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),                   
                    drivetrain.createTrajectoryCommand(seg1),
                    drivetrain.createTrajectoryCommand(seg2),
                    new ParallelDeadlineGroup(
                        new ShootCommand(ball_handling),
                        new StayPutCommand(drivetrain)
                    ),
                    new ParallelDeadlineGroup(
                        new ShootCommand(ball_handling),
                        new StayPutCommand(drivetrain)
                    )));
        }

        {   // TODO waypoints Right Tarmac, Left Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false, -1.045, 0.5, 10);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(2.16, -0.73, -10));
    
            auto_options.addOption("RTLEPickShootShoot",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", false),
                    new ApplySettingCommand("SpinnerSetpoint", 65),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),          
                    drivetrain.createTrajectoryCommand(seg1),
                    drivetrain.createTrajectoryCommand(seg2),
                    new ParallelDeadlineGroup(
                        new ShootCommand(ball_handling),
                        new StayPutCommand(drivetrain)
                    ),
                    new ParallelDeadlineGroup(
                        new ShootCommand(ball_handling),
                        new StayPutCommand(drivetrain)
                    )));
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