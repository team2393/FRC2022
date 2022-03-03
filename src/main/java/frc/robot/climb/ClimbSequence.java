// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Command sequence to half-automated climb */
public class ClimbSequence extends SequentialCommandGroup
{
    public ClimbSequence(final Climber climber)
    {
        setName("Climb Sequence");
        addCommands(
            // Up, then drive up to first rung
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm High", 0.85)),

            // Lift up to first rung
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm Low", -0.04)),

            // Hang on 1st rung
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm Mid", 0.4)),

            // Stretch out to 2nd rung: Tilt..   
            new ArmInOutCommand(climber, true),
            new WaitForNextStepCommand(),
            // reach out...
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm High", 0.85)),
            // latch onto 2nd
            new ArmInOutCommand(climber, false),
            new WaitForNextStepCommand(),

            // Pull up to 2nd
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm Low", -0.04)),

            // Hang on 2nd rung
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm Mid", 0.4)),

            // Stretch out to 3rd rung: Tilt..   
            new ArmInOutCommand(climber, true),
            new WaitForNextStepCommand(),
            // reach out...
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm High", 0.85)),
            // latch onto 3rd
            new ArmInOutCommand(climber, false),
            new WaitForNextStepCommand(),

            // Pull up to 3rd
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm Low", -0.04)),

            // Hang on 3rd rung
            new ParallelDeadlineGroup(
                new WaitForNextStepCommand(),
                new SetClimberExtensionCommand(climber, "Arm Mid", 0.4))
        );
    }
}
