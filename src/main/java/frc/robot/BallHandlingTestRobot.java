// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cargo.BallHandling;

/** Very simple robot that tests the spinner */
public class BallHandlingTestRobot extends TimedRobot
{
    private final BallHandling ballhandling = new BallHandling();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Ball Handling Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();        
    }

    @Override
    public void disabledInit()
    {
        ballhandling.enable(false);
    }

    @Override
    public void teleopInit()
    {
        OperatorInterface.reset();
        ballhandling.enable(true);
    }

    @Override
    public void teleopPeriodic()
    {
        if (OperatorInterface.doShoot())
            ballhandling.shoot();

        if (OperatorInterface.toggleSpinner())
            ballhandling.toggleSpinner();


        if (OperatorInterface.toggleLoading())
            ballhandling.toggleLoading();
    }
}
