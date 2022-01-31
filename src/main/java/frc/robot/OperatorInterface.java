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

    public static void reset()
    {
        // Call all the 'pressed' methods that we use once
        // so they start out 'false', not remmebering some past presses
        joystick.getAButtonPressed();
        joystick.getXButtonPressed();
        joystick.getRightBumperPressed();
    }

    /** @return Speed that driver requests, -1..1, positive is "forward" (Left stick for/back) */
    public static double getSpeed()
    {
        if (joystick.getRightTriggerAxis() > 0.5)
            return -joystick.getLeftY() / 2;
        return -joystick.getLeftY();
    }

    /** @return Rotation that driver requests, -1..1, positive is "right" or "clockwise" (Right stick left/right) */
    public static double getRotation()
    {
        if (joystick.getRightTriggerAxis() > 0.5)
            return joystick.getRightX() / 2;
        return joystick.getRightX();
    }

    /** @return Do we want to shoot? (Left bumper) */
    public static boolean doShoot()
    {
        return joystick.getLeftBumper();
    }

    /** @return Do we want to always run the spinner? Toggles on each press (X) */
    public static boolean toggleSpinner()
    {
        return joystick.getXButtonPressed();
    }

    /** @return Toggle load/don't load (A) */
    public static boolean toggleLoading()
    {
        return joystick.getAButtonPressed();
    }

    /** @return Shift into high gear?  (POV up) */
    public static boolean shiftHigh()
    {
        // Push POV up, "north"
        return joystick.getPOV() == 0;
    }

    /** @return Shift into low gear? (POV down) */
    public static boolean shiftLow()
    {
        // Push POV down, "south"
        return joystick.getPOV() == 180;
    }

    /** @return Toggle climbing arm down/up (right bumper) */
    public static boolean toggleArmAngle()
    {
        return joystick.getRightBumperPressed();
    }

    /** @return Extend arm, positive for 'out' (Right stick for/back) */
    public static double extendArm()
    {
        return -joystick.getRightY();
    }
}
