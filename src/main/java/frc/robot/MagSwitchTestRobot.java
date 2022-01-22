// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Very simple robot that tests REV magnetic limit switch */
public class MagSwitchTestRobot extends TimedRobot
{
    /** Sensor, connected to DIO on RoboRIO
     * 
     *  Switch black         = GND,
     *         red           = 5V
     *         white or blue = Signal
     */
    private final DigitalInput magswitch = new DigitalInput(7);
        
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 REV Magnetic Limit Switch Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        // Switch should be fail-safe
        //
        // If we're not at the limit, i.e. magnet is not at the sensor, it reports 'true'.
        // If we are at the limit, i.e. magnet is close to the sensor,
        // OR there's a break in the cable, the sensor is disconnected, it reports 'false'
        if (magswitch.get())
            SmartDashboard.putString("Mag Switch", "Open");
        else
            SmartDashboard.putString("Mag Switch", "At limit!");
    }
}
