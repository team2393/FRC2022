// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.cargo.CargoSensor;

/** Robot that tests the cargo sensor */
public class CargoSensorTestRobot extends TimedRobot
{
    private final CargoSensor sensor = new CargoSensor();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Cargo Sensor Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        sensor.read();
    }
}
