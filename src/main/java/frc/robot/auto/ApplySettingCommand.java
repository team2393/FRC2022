// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Command that applies one settings */
public class ApplySettingCommand extends InstantCommand
{
    public ApplySettingCommand(final String name, final double value)
    {
        super(() -> SmartDashboard.putNumber(name, value));
    }

    public ApplySettingCommand(final String name, final boolean value)
    {
        super(() -> SmartDashboard.putBoolean(name, value));
    }
}