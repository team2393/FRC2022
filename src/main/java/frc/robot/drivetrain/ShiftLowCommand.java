// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Command that shifts low */
public class ShiftLowCommand extends InstantCommand
{
    public ShiftLowCommand(final Drivetrain drivetrain)
    {
        super(() -> drivetrain.shiftgear(false));
    }
}
