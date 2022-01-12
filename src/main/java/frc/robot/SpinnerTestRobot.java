// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/** Very simple robot that tests the spinner */
public class SpinnerTestRobot extends TimedRobot
{
    private final Spinner spinner = new Spinner();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Spinner Test *****");
    }

    @Override
    public void teleopPeriodic()
    {
        // In teleop, manually control spinner through joystick 
        spinner.setSpeed(OperatorInterface.getSpeed());
    }
}
