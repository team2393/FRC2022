// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.wpilibj.XboxController;

/** Everything related to the 'operator interface'
 * 
 *  Helpers for reading joystick and buttons
 */
public class OperatorInterface
{
    /** Controller used by primary driver */
    private final static XboxController joystick = new XboxController(0);

    /** @return Speed that driver requests, -1..1, positive is "forward" */
    public static double getSpeed()
    {
        if (joystick.getRightTriggerAxis() > 0.5)
            return -joystick.getLeftY() / 2;
        return -joystick.getLeftY();
    }

    /** @return Rotation that driver requests, -1..1, positive is "right" or "clockwise" */
    public static double getRotation()
    {
        if (joystick.getRightTriggerAxis() > 0.5)
            return joystick.getRightX() / 2;
        return joystick.getRightX();
    }
}
