// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** 'Main' is where the robot code starts.
 *  Used to launch one of the actual XXXRobot classes.
 */
public final class Main
{
    /** Change to select which robot code you want to run */
    public static void main(String... args)
    {
        RobotBase.startRobot(SpinnerTestRobot::new);
    }
}
