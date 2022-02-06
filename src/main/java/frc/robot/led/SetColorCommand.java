// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Command that sets a single color */
public class SetColorCommand extends InstantCommand
{
    public SetColorCommand(final LEDStrip strip,
                           final Color color)
    {
        super(() -> strip.setAll(color), strip);
    }

    public SetColorCommand(final LEDStrip strip,
                           final int red, final int green, final int blue)
    {
        super(() -> strip.setAll(red, green, blue), strip);
    }
}
