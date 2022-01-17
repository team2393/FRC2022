// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.camera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.ApplySettingsCommand;

/** Helper for dealing with camera on Raspberry Pi */
public class CameraHelper
{
    /** Register dashboard commands */
    public static void registerCommands()
    {
        SmartDashboard.putData(new ApplySettingsCommand("Red Cargo", "detect_red.dat"));
        SmartDashboard.putData(new ApplySettingsCommand("Blue Cargo", "detect_blue.dat"));
    }    
}
