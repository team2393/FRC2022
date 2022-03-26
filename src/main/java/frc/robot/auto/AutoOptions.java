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

        {   // "L" test of rotating in place
            // Run forward 1.5 m
            Trajectory seg1 = TrajectoryHelper.createTrajectory(1.5, 0, 0);

            // Rotate to 90 degrees in place
            Pose2d seg1_rot = TrajectoryHelper.rotateInPlaceToHeading(seg1, 90);

            // Move to the left
            Trajectory seg2 = TrajectoryHelper.createTrajectory(seg1_rot, 1.5, 1, 90);

            // Rotate to atan2(1, 1.5) = 33.67 degrees in place, so back points straight back to origin
            Pose2d seg2_rot = TrajectoryHelper.rotateInPlaceToHeading(seg2, 33.67);

            // Back to origin
            Trajectory seg3 = TrajectoryHelper.createTrajectory(seg2_rot, false, 0, 0, 0);

            auto_options.addOption("L (inverted) test",
                new SequentialCommandGroup(
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(seg1),
                    new RotateToHeadingCommand(drivetrain, 90.0),
                    drivetrain.createTrajectoryCommand(seg2),
                    new RotateToHeadingCommand(drivetrain, 33.67),
                    drivetrain.createTrajectoryCommand(seg3)       ));
        }


        {   // Right Tarmac Middle Edge 4 ball
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false, -0.96, 0, -9.4);
            Pose2d seg1_rot = TrajectoryHelper.rotateInPlaceToHeading(seg1, -114);
            Trajectory seg2 = TrajectoryHelper.createTrajectory(seg1_rot, false,
                                                                0.02, 2.7, -106,
                                                                -0.43, 5.8, -66);
            Trajectory seg3 = TrajectoryHelper.createTrajectory(seg2, 0.65, 3.18, -61.0);
            
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
                    new ApplySettingCommand("HoodSetpoint", 54.1),
                    new ApplySettingCommand("SpinnerSetpoint", 40.4),
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

        {   // Left Tarmac, Middle Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.2, 0, 6.5);
    
            auto_options.addOption("LTMEPickShootShoot (CTFDU)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", true),
                    new ApplySettingCommand("SpinnerSetpoint", 70),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),                   
                    drivetrain.createTrajectoryCommand(seg1),
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

        {   // Left Tarmac, Right Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.4, -1.2, 50,
                                                                      -1.5, -1.3, 8.5);
    
            auto_options.addOption("LTREPickShootShootFar (Hamburger)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", true),
                    new ApplySettingCommand("SpinnerSetpoint", 72),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),                   
                    drivetrain.createTrajectoryCommand(seg1),
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

        {   // Left Tarmac, Left Edge, pickup, shoot from far
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false,-1.5, 0.01, 8.4);
    
            auto_options.addOption("LTLETaxiShootFar (GG)",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", true),
                    new ApplySettingCommand("SpinnerSetpoint", 70),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    drivetrain.createTrajectoryCommand(seg1),
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

        {   // Right Tarmac, Left Edge, pickup, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false, -1.08, -0.01, -2.3);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(1.96, 0.75, -28));
            Trajectory seg3 = TrajectoryHelper.continueTrajectory(seg2,
                TrajectoryHelper.createTrajectory(false, -1.85, 1, -21));
            
    
            auto_options.addOption("RTMEPickShootShootPickShoot",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", true),
                    new ApplySettingCommand("SpinnerSetpoint", 72),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),          
                    drivetrain.createTrajectoryCommand(seg1),
                    new ShootCommand(ball_handling),
                    new WaitCommand(1.5),
                    new ShootCommand(ball_handling),
                    drivetrain.createTrajectoryCommand(seg2),
                    new ApplySettingCommand("SpinnerSetpoint", 73),
                    new OpenIntakeCommand(ball_handling),  
                    drivetrain.createTrajectoryCommand(seg3), 
                    new ShootCommand(ball_handling)));
        }

        {   // Right Tarmac, Left Edge, pickup, shoot, shoot, pick, pick, shoot, shoot
            Trajectory seg1 = TrajectoryHelper.createTrajectory(false, -1.27, -0.1, 10);
            Trajectory seg2 = TrajectoryHelper.continueTrajectory(seg1,
                TrajectoryHelper.createTrajectory(false, -3.45, 1.73, 10));
            Trajectory seg3 = TrajectoryHelper.continueTrajectory(seg2,
                TrajectoryHelper.createTrajectory(2.47, -1.27, -19));
            
    
            auto_options.addOption("RTLEPSSPSS",
                new SequentialCommandGroup(
                    new ApplySettingCommand("High", true),
                    new ApplySettingCommand("SpinnerSetpoint", 72),
                    new ToggleSpinnerCommand(ball_handling),
                    new ShiftLowCommand(drivetrain),
                    new OpenIntakeCommand(ball_handling),          
                    drivetrain.createTrajectoryCommand(seg1),
                    new ShootCommand(ball_handling),
                    new WaitCommand(.5),
                    new ShootCommand(ball_handling),
                    new OpenIntakeCommand(ball_handling),  
                    drivetrain.createTrajectoryCommand(seg2),
                    new ApplySettingCommand("SpinnerSetpoint", 81),
                    drivetrain.createTrajectoryCommand(seg3), 
                    new ShootCommand(ball_handling),
                    new WaitCommand(.5),
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