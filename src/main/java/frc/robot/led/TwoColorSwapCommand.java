// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.led.LEDStrip.N;

/** Command that shows color 1, 2, 1, 2, ... and then 2, 1, 2, 1 */
public class TwoColorSwapCommand extends CommandBase
{
    private final LEDStrip strip;
    private final double step_secs;
    private final Color color1, color2;

    /** @param strip Strip to use
     *  @param step_secs Duration of each step
     *  @param color1 Two colors..
     *  @param color2
     */
    public TwoColorSwapCommand(final LEDStrip strip,
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
    public void execute()
    {
        if (Math.round(Timer.getFPGATimestamp() / step_secs) % 2 == 0)
        {
            for (int i=0; i<N; i+=2)
                strip.set(i, color1);
            for (int i=1; i<N; i+=2)
                strip.set(i, color2);
        }
        else
        {
            for (int i=0; i<N; i+=2)
                strip.set(i, color2);
            for (int i=1; i<N; i+=2)
                strip.set(i, color1);
        }
    }
}
