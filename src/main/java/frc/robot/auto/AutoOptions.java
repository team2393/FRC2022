     // Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.drivetrain.RotateToHeadingCommand;
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

        {   // Right Tarmac Middle Edge 4 ball
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false, -0.96, 0, -9.4);
            Pose2d seg1_rot = TrajectoryHelper.rotateInPlaceToHeading(seg1, -114);
            Trajectory seg2 = TrajectoryHelper.createTrajectory(seg1_rot, false,
                                                                0.02, 2.7, -106,
                                                                -0.43, 5.8, -66);
            Trajectory seg3 = TrajectoryHelper.createTrajectory(seg2, 0.16, 4.37, -63.7);
            
            auto_options.addOption("RTME4Ball",
                new SequentialCommandGroup(
                    new ApplySettingCommand("HoodSetpoint", 65.6),
                    new ApplySettingCommand("SpinnerSetpoint", 35),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),     
                    drivetrain.createTrajectoryCommand(seg1),
                    new ShootCommand(ball_handling),
                    new WaitCommand(.5),
                    new ShootCommand(ball_handling),
                    new RotateToHeadingCommand(drivetrain, -114),
                    new OpenIntakeCommand(ball_handling),     
                    drivetrain.createTrajectoryCommand(seg2),
                    new ApplySettingCommand("HoodSetpoint", 86.5),
                    new ApplySettingCommand("SpinnerSetpoint", 42.7),
                    drivetrain.createTrajectoryCommand(seg3), 
                    new ShootCommand(ball_handling),
                    new WaitCommand(.5),
                    new ShootCommand(ball_handling)));


        }

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

        {   // Right Tarmac, Left Edge, pickup, shoot, shoot, pick, pick human, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false, -1.83, -0.03, 10.28);
            Trajectory seg2 = TrajectoryHelper.createTrajectory(seg1, false, -4.94, 0.98, 8.26);
            Trajectory seg3 = TrajectoryHelper.createTrajectory(seg2, -2.3, 1.04, -1.5);            
    
            auto_options.addOption("RTLE4BallHuman",
                new SequentialCommandGroup(
                    new ApplySettingCommand("HoodSetpoint", 57.3),
                    new ApplySettingCommand("SpinnerSetpoint", 39.07),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),          
                    drivetrain.createTrajectoryCommand(seg1),
                    new ShootCommand(ball_handling),
                    new WaitCommand(.5),
                    new ShootCommand(ball_handling),
                    new OpenIntakeCommand(ball_handling),  
                    drivetrain.createTrajectoryCommand(seg2),
                    new ApplySettingCommand("SpinnerSetpoint", 41.4),
                    new ApplySettingCommand("HoodSetpoint", 51.6),
                    drivetrain.createTrajectoryCommand(seg3), 
                    new ShootCommand(ball_handling),
                    new WaitCommand(.5),
                    new ShootCommand(ball_handling)));
        }

        {   // Left Tarmac, Middle Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.17, -0.01, 9.4);
    
            auto_options.addOption("LTMEPickShootShoot (CTFDU)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("HoodSetpoint", 66),
                    new ApplySettingCommand("SpinnerSetpoint", 36.1),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),                   
                    new WaitCommand(1.5), // assert intake is ope, hood positioned, ...
                    drivetrain.createTrajectoryCommand(seg1),
                    new ShootCommand(ball_handling),
                    new WaitCommand(1.5),
                    new ShootCommand(ball_handling)));
        }

        {   // Left Tarmac, Right Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.59, -1.4, 25,
                                                                      -2.1,-1.7,15);
            auto_options.addOption("LTREPickShootShoot (Hamburger)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("HoodSetpoint", 55),
                    new ApplySettingCommand("SpinnerSetpoint", 40),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),                   
                    drivetrain.createTrajectoryCommand(seg1),
                    new ShootCommand(ball_handling),
                    new WaitCommand(1.5),
                    new ShootCommand(ball_handling)
                    ));
        }

        {   // Left Tarmac, Left Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.52, -0.01, 9.7);
    
            auto_options.addOption("LTLETaxiShoot (GG)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("HoodSetpoint", 63),
                    new ApplySettingCommand("SpinnerSetpoint", 37),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(seg1),
                    new ShootCommand(ball_handling)));
        }


        {   // Right Tarmac, Left Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false, -1.3, -0.2, 14);
    
            auto_options.addOption("RTLEPickShootShoot (SM)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("HoodSetpoint", 69),
                    new ApplySettingCommand("SpinnerSetpoint", 36),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),          
                    new WaitCommand(1.5),
                    drivetrain.createTrajectoryCommand(seg1),
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