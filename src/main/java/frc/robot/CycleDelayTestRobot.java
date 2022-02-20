// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.util.CycleDelayFilter;

/** Robot to test CycleDelayFilter */
public class CycleDelayTestRobot extends TimedRobot
{
    // Expected to give 2-cycle delay
    private final CycleDelayFilter delay = CycleDelayFilter.forSeconds(0.04);
    
    @Override
    public void teleopPeriodic()
    {
        // Raw signal changes every half second
        final boolean raw = (RobotController.getFPGATime() / 500000) % 2 == 0;
        final boolean delayed = delay.compute(raw);
        System.out.println(raw + " -> " + delayed);
    }
}