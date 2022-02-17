// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Simple robot that tests fiber optic ball detector */
public class BallDetectorTestRobot extends TimedRobot
{
    /** Sensor, connected to DIO on RoboRIO
     * 
     *  Detector black   ->  Digital Input,
     *           brown   ->  12V
     *           blue    ->  Gnd
     */
    private final DigitalInput fiberopticswitch = new DigitalInput(RobotMap.FEEDER_SENSOR);
        
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Ball Detector Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        // Digital inputs have an internal 'pull up' resistor,
        // so with nothing connected they read 'true'.
        // The sensor, if it detects light, connects the digital input to ground,
        // resulting in a reading of 'false'.
        //
        // false: Light detected, no ball in path of light
        // true:  Nothing connected, or no light, i.e., ball in path of light
        //
        // This is fail-safe in the sense that we want to stop moving
        // conveyors etc. when we detect a ball.
        // If the sensor becomes disconnected (broken wire, ...)
        // that will also look like a ball and also stop the conveyor.
        // We "err" on the side of stopping the conveyor.
        // Same could be achieved using a "normally closed" switch
        // that connects the input to ground and opens when touched
        // by a ball.
        //
        // To operate the conveyor while we don't have a sensor,
        // we need to install a plug that connects the input to ground.
        if (fiberopticswitch.get())
            SmartDashboard.putString("Ball Detector", "Ball?");
        else
            SmartDashboard.putString("Ball Detector", "No Ball!");
    }
}
