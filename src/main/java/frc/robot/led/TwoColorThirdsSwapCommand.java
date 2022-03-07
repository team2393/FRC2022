// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.led.LEDStrip.N;

/** Command that shows color 1 2 1, then 2 1 2 in thirds of strip */
public class TwoColorThirdsSwapCommand extends CommandBase
{
    private final LEDStrip strip;
    private final double step_secs;
    private final Color color1, color2;

    /** @param strip Strip to use
     *  @param step_secs Duration of each step
     *  @param color1 Two colors..
     *  @param color2
     */
    public TwoColorThirdsSwapCommand(final LEDStrip strip,
                               final double step_secs,
                               final Color color1,
                               final Color color2)
    {
        this.strip = strip;
        this.step_secs = step_secs;
        this.color1 = color1;
        this.color2 = color2;
        addRequirements(strip);
    }

    @Override
    public boolean runsWhenDisabled()
    {
        return true;
    }

    @Override
    public void execute()
    {
        final Color a, b;
        if (Math.round(Timer.getFPGATimestamp() / step_secs) % 2 == 0)
        {
            a = color1;
            b = color2;
        }
        else
        {
            a = color2;
            b = color1;
        }
        int i = 0;
        while (i < N/3)
            strip.set(i++, a);
        while (i < 2*N/3)
            strip.set(i++, b);
        while (i < N)
            strip.set(i++, a);
    }
}
