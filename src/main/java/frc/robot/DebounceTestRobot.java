// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Very simple robot that blinks LED */
public class DebounceTestRobot extends TimedRobot
{
    private final XboxController joystick = new XboxController(0);
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 debounce Test *****");
    }

    private final Debouncer debounce = new Debouncer(5.0, DebounceType.kRising);

    @Override
    public void teleopPeriodic()
    {
        final boolean input = joystick.getXButton();
        final boolean output = debounce.calculate(input);
        SmartDashboard.putNumber("input", input ? 1 : 0);
        SmartDashboard.putNumber("output", output ? 1 : 0);
        NetworkTableInstance.getDefault().flush();
    }
}
