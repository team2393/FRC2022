// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cargo.Spinner2;

/** Robot that tests the spinner using Falcon's built-in speed control loop */
public class SpinnerFalconTestRobot extends TimedRobot
{
    private final Spinner2 spinner = new Spinner2();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Spinner Falcon Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        
        SmartDashboard.putBoolean("Spinner Ball Ejected", spinner.isBallEjected());
        SmartDashboard.putNumber("Spinner RPM", spinner.getSpeed() * 60.0);
        SmartDashboard.putNumber("Spinner Rev", spinner.getPosition());

        // Send out on each period to get better plots
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void teleopInit()
    {
        spinner.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        // In teleop, manually control spinner joystick
        spinner.setOutput(OperatorInterface.getSpeed());
    }

    @Override
    public void autonomousPeriodic()
    {
        // In auto, ask spinner to run at desired speed
        spinner.run();

        final double setpoint = spinner.getSetpoint();
        if (setpoint != 0)
            SmartDashboard.putNumber("Error", (setpoint - spinner.getSpeed()) / setpoint);
        else
            SmartDashboard.putNumber("Error", 0.0);
    }
}
