// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import edu.wpi.first.wpilibj.TimedRobot;

/** Filter that delays a "true" signal for a number of cycles */
public class CycleDelayFilter
{
    private int delay_cycles;
    private int waited = 0;

    /** @param seconds Seconds to delay a true signal
     *  @return Filter that delays by appropriate number of cycles
     */
    public static CycleDelayFilter forSeconds(final double seconds)
    {
        // Can only delay by full cycle periods; round down
        return new CycleDelayFilter((int) (seconds / TimedRobot.kDefaultPeriod));
    }

    /** Create a delay of 'true' for N cycles
     *  @param delay_cycles Number of 50 Hz (20 ms) robot cycles to delay
     */
    public CycleDelayFilter(final int delay_cycles)
    {
        this.delay_cycles = delay_cycles;  
    }

    /** Compute value, to be called in each TimedRobot period
     *  @param input Original true/false signal
     *  @return Signal where true is delayed by N cycles
     */
    public boolean compute(final boolean input)
    {
        if (input)
        {   // Did input just turn true and we need to delay?
            if (waited < delay_cycles)
            {
                ++waited;
                return false;
            }
            // Past delay, indicate true
            return true;
        }
        // input is false: Reset delay and return unmodified input
        waited = 0;
        return false;
    }
}