// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Very simple robot that tests fiber optic ball detector */
public class BallDetectorTestRobot extends TimedRobot
{
    /** Sensor, connected to DIO on RoboRIO
     * 
     *  Detector black   = Digital Input,
     *           brown   = 12V
     *           blue    = Gnd
     */
    private final DigitalInput fiberopticswitch = new DigitalInput(7);
        
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Ball Detector Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        // false: Light, no ball
        // true:  Not connected, no light, ball in path of light
        if (fiberopticswitch.get())
            SmartDashboard.putString("Ball Detector", "Ball?");
        else
            SmartDashboard.putString("Ball Detector", "No Ball!");
    }
}
