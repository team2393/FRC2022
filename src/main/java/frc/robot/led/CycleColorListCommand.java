// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that cycles through a list of colors */
public class CycleColorListCommand extends CommandBase
{
    private final LEDStrip strip;
    private final double step_secs;
    private final Color[] colors;
    private final Timer timer = new Timer();
    private int current;

    /** @param strip Strip to use
     *  @param step_secs Duration of each color
     *  @param colors One or more colors
     */
    public CycleColorListCommand(final LEDStrip strip,
                                 final double step_secs,
                                 final Color... colors)
    {
        this.strip = strip;
        this.step_secs = step_secs;
        this.colors = colors;
        addRequirements(strip);
    }

    @Override
    public void initialize()
    {
        // Show first color
        current = 0;
        strip.setAll(colors[current]);

        // (Re-)start timer
        timer.stop();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute()
    {
        if (timer.advanceIfElapsed(step_secs))
        {
            // Show next color
            current = (current + 1) % colors.length;
            strip.setAll(colors[current]);
        }
    }
}
