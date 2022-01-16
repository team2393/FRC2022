// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Very simple robot for pigeon tests */
public class PigeonTestRobot extends TimedRobot
{
    private final PigeonIMU pigeon = new PigeonIMU(0);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Pigeon Test *****");

        pigeon.configFactoryDefault();
        pigeon.clearStickyFaults();
    }

    @Override
    public void robotPeriodic()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("Heading", pigeon.getFusedHeading());
        SmartDashboard.putNumber("Yaw", ypr[0]);
        SmartDashboard.putNumber("Pitch", ypr[1]);
        SmartDashboard.putNumber("Roll", ypr[2]);
        SmartDashboard.putNumber("Temperature", pigeon.getTemp());
    }

    @Override
    public void teleopInit()
    {
        pigeon.setFusedHeading(0.0);
        pigeon.setYaw(0.0);
    }
}
