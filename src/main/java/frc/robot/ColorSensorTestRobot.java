// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/** Very simple robot that tests the color sensor */
public class ColorSensorTestRobot extends TimedRobot
{
    /** Threshold for recognizing color, 0..1 */
    private static final double THRESHOLD = 0.8;

    /** Sensor, connected to I2C socket on RIO */
    private final ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);

    /** Values for red and blue cargo balls
     *  with light on sensor on.
     *  TODO Detects "nothing" as "red"
     *  Need to position sensor such that it sees e.g. black when there's no ball 
     */
    private final Color red = new Color(0.4, 0.4, 0.2);
    private final Color blue = new Color(0.2, 0.45, 0.34);

    private final ColorMatch match = new ColorMatch();
    
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Color Sensor Test *****");

        // Tell matcher to look for these two colors, 80% accuracy
        match.addColorMatch(red);
        match.addColorMatch(blue);
        match.setConfidenceThreshold(THRESHOLD);
    }

    @Override
    public void robotPeriodic()
    {
        // Get and show RGB
        final Color color = sensor.getColor();
        SmartDashboard.putNumber("R", color.red);
        SmartDashboard.putNumber("G", color.green);
        SmartDashboard.putNumber("B", color.blue);

        // See if it's close enough to a ball color
        ColorMatchResult check = match.matchClosestColor(color);
        if (check == null)
            SmartDashboard.putString("Color", "null");
        else if (check.confidence >= THRESHOLD  &&  check.color.equals(red))
            SmartDashboard.putString("Color", String.format("Red %.1f", check.confidence));
        else if (check.confidence >= THRESHOLD  &&  check.color.equals(blue))
            SmartDashboard.putString("Color", String.format("Blue %.1f", check.confidence));
        else
            SmartDashboard.putString("Color", "??!??");
    }
}
