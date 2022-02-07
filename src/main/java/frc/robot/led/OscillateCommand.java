// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.led.LEDStrip.N;

/** Command that oscillates some green section up and down over a light blue background */
public class OscillateCommand extends CommandBase
{
    private final LEDStrip strip;

    public OscillateCommand(final LEDStrip strip)
    {
        this.strip = strip;
        addRequirements(strip);
    }

    @Override
    public void execute()
    {
        final int size = 5;
        final double phase = 2*Math.PI * Timer.getFPGATimestamp();
        final int pos = N/2 + (int) ((N/2 - size + 1) * Math.sin(phase));

        // Background color
        strip.setAll(0, 0, 10);

        // Moving green section
        int start, end;
        if (pos > N/2)
        {
            start = MathUtil.clamp(pos, N/2, N);
            end = MathUtil.clamp(pos + size, N/2, N);
        }
        else
        {
            start = MathUtil.clamp(pos-size, 0, N/2);
            end = MathUtil.clamp(pos, 0, N/2);
        }
        for (int i=start; i<end; ++i)
            strip.set(i, 255, 255, 10);
    }
}
