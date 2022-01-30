// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/** Filter that keeps a boolean "true" for some time
 * 
 *  Used to remember some brief event for some time.
 */
public class KeepOnFilter
{
    private double duration;
    private double keep_on_until = 0.0;

    public KeepOnFilter(final double duration)
    {
        this.duration = duration;  
    }

    public boolean compute(final boolean input)
    {
        final double now = Timer.getFPGATimestamp();
        if (input)
        {
            keep_on_until = now + duration;
            return true;
        }
        return now <= keep_on_until;
    }
}