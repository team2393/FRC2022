// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;

/** Very simple robot that blinks LED */
public class LEDTestRobot extends TimedRobot
{
    private final DigitalOutput led = new DigitalOutput(4);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 LED Test *****");
    }

    @Override
    public void disabledPeriodic()
    {
        // Digital output can be controlled even while disabled
        // and disconnected from driver station.
        // Toggle LED on/off every two seconds
        led.set((System.currentTimeMillis() / 2000) % 2 == 0);
    }

    @Override
    public void teleopPeriodic()
    {
        // In teleop, manually turn LED on/off via the 'USER' button
        led.set(RobotController.getUserButton());
    }

    @Override
    public void autonomousPeriodic()
    {
        // In autonomous, toggle LED on/off every half second
        led.set((System.currentTimeMillis() / 500) % 2 == 0);
    }
}
