// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;

/** Very simple robot for testing the new REV Robotics Pneumatic and power Hub */
public class PneumaticHubTestRobot extends TimedRobot
{
    private PowerDistribution power = new PowerDistribution();

    // private final PneumaticHub hub = new PneumaticHub();
    private List<Solenoid> solenoids = new ArrayList<>();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Pneumatic Hub Test *****");

        System.out.println("Power distribution board channels: " + power.getNumChannels());

        // Create one solenoid for each channel
        for (int ch=0; ch<16; ++ch)
            solenoids.add(new Solenoid(PneumaticsModuleType.REVPH, ch));
    }

    @Override
    public void autonomousPeriodic()
    {
        // Every second, toggle another of the channels 0..15
        final int ch = (int) (System.currentTimeMillis() / 1000) % 16;
        solenoids.get(ch).toggle();

        // Every 10 seconds, toggle the switchable channel on the Rev power hub
        power.setSwitchableChannel((System.currentTimeMillis() / 10000) % 2 == 1);
    }
}
