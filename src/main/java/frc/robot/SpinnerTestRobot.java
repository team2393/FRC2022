// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cargo.Spinner;

/** Very simple robot that tests the spinner */
public class SpinnerTestRobot extends TimedRobot
{
    private final Spinner spinner = new Spinner();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Spinner Test *****");

        SmartDashboard.setDefaultNumber("Spinner Setpoint", 0.0);
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        
        // Spinner itself displays speed on dashboard.
        // Add details needed to configure/tune the spinner.
        SmartDashboard.putNumber("Spinner Rev", spinner.getPosition());
        SmartDashboard.putNumber("Spinner Voltage", spinner.getVoltage());
    }

    @Override
    public void teleopInit()
    {
        spinner.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        // In teleop, manually control spinner voltage -12..12 through joystick
        double voltage = 12.0*OperatorInterface.getSpeed();
        spinner.setVoltage(voltage);
    }

    @Override
    public void autonomousPeriodic()
    {
        // In auto, ask spinner to run at desired speed
        // spinner.setVoltage(SmartDashboard.getNumber("Spinner Setpoint", 0.0));
        spinner.setSpeed(SmartDashboard.getNumber("Spinner Setpoint", 0.0));
    }
}
