// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.led.LEDStrip.N;

/** Command that 'bursts' a color from the center */
public class BurstCommand extends CommandBase
{
    private final LEDStrip strip;
    private final double period;
    private final Color color;

    /** @param strip LEDStrip
     *  @param period Seconds between steps
     *  @param color Color to use
     */
    public BurstCommand(final LEDStrip strip, final double period, final Color color)
    {
        this.strip = strip;
        this.period = period;
        this.color = color;
        addRequirements(strip);
    }

    @Override
    public void execute()
    {
        // Start at center (N/2) and move to ends (0 resp. N)
        // --> 'step' is distance from center, 0 to N/2
        int step = (int) (Math.round(Timer.getFPGATimestamp()/period) % (N/2));

        strip.setAll(Color.kBlack);
        strip.set(MathUtil.clamp(N/2-step, 0, N/2), color);
        strip.set(MathUtil.clamp(N/2+step, N/2, N-1), color);
    }
}
