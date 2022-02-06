// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.led.LEDStrip.N;

/** Command that smoothly cycles through the rainbow */
public class RainbowCommand extends CommandBase
{
    private final LEDStrip strip;

    /** Seconds for one rainbow cycle */
    private static final double cycle_secs = 3.0;

    public RainbowCommand(final LEDStrip strip)
    {
        this.strip = strip;
        addRequirements(strip);
    }

    @Override
    public void execute()
    {
        final double seconds = Timer.getFPGATimestamp();

        // One cycle means 180 degrees of hue.
        // Every cycle_secs, move the start hue by 180
        final int start_hue = (int) (Math.round(seconds / cycle_secs * 180.0) % 180);

        // Set first LED to start hue, rest so that we run through 180 degrees
        for (int i=0; i<N; ++i)
        {
            int hue = (start_hue + (i * 180 / N)) % 180;
            strip.set(i,
                      Color.fromHSV(hue, 255, 128));
        }
    }
}
