// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;

/** Indicate 'speed' (forward, backward) via LED strip */
public class SpeedIndicatorCommand extends CommandBase
{
    private LEDStrip strip;

    public SpeedIndicatorCommand(LEDStrip strip)
    {
        this.strip = strip;
    }

    @Override
    public void execute()
    {
        double speed = OperatorInterface.getSpeed();

        strip.setAll(Color.kBlack);

        if (speed > 0)
        {
            int count = (int) (15*speed);
            for (int i=15;  i < 15+count; ++i)
                strip.set(i, Color.kGreen);
        }
        else if (speed < 0)
        {
            int count = (int) (-15*speed);
            for (int i=15;  i > 15-count; --i)
                strip.set(i, Color.kRed);
        }


    }
}
