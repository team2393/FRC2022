// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.camera.CameraHelper;
import frc.robot.camera.GuessingUDPClient;

/** Very simple robot for camera tests */
public class CameraTestRobot extends TimedRobot
{
    /** Servo that rotates camera */
    private final Servo rotator = new Servo(0);
    private GuessingUDPClient guesser;

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Camera Test *****");

        CameraHelper.registerCommands();

        SmartDashboard.setDefaultNumber("rotate", 90.0);
        SmartDashboard.setDefaultNumber("rotate gain", 0.0);

        try
        {
            guesser = new GuessingUDPClient();
        }
        catch (Exception ex)
        {
            throw new RuntimeException(ex);
        }
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic()
    {
        rotator.setAngle(SmartDashboard.getNumber("rotate", 90.0));
    }
    
    @Override
    public void autonomousInit()
    {
        rotator.setAngle(90.0);
    }

    @Override
    public void autonomousPeriodic()
    {
        // Pick one:
        final double direction = SmartDashboard.getNumber("Direction", 0);
        // final double direction = guesser.get().direction;

        final double current_heading = rotator.getAngle();
        double new_heading = current_heading - direction * SmartDashboard.getNumber("rotate gain", 0.0);
        new_heading = MathUtil.clamp(new_heading, 5.0, 175.0);
        SmartDashboard.putNumber("rotate", new_heading);
        rotator.setAngle(new_heading);
    }
}
