// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.ApplySettingsCommand;

/** Very simple robot for camera tests */
public class CameraTestRobot extends TimedRobot
{
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Camera Test *****");

        SmartDashboard.putData(new ApplySettingsCommand("Red Cargo", "detect_red.dat"));
        SmartDashboard.putData(new ApplySettingsCommand("Blue Cargo", "detect_blue.dat"));
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }
}
