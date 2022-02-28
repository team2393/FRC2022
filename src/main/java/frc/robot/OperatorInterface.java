// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/** Everything related to the 'operator interface'
 * 
 *  Helpers for reading joystick and buttons
 */
public class OperatorInterface
{
    /** Controller used by primary driver */
    public static final XboxController joystick = new XboxController(0);
    public static final GenericHID buttons = new GenericHID(1);

    public static void reset()
    {
        // Call all the 'pressed' methods that we use once
        // so they start out 'false', not remmebering some past presses
        joystick.getAButtonPressed();
        joystick.getBButtonPressed();
        joystick.getXButtonPressed();
        joystick.getYButtonPressed();
        joystick.getRightBumperPressed();

        for (int i=1; i<9; ++i)
            buttons.getRawButtonPressed(i);
    }

    /** @return Speed that driver requests, -1..1, positive is "forward" (Left stick for/back) */
    public static double getSpeed()
    {
        if (joystick.getLeftTriggerAxis() > 0.5)
            return -joystick.getLeftY() / 2;
        return -joystick.getLeftY();
    }

    /** @return Rotation that driver requests, -1..1, positive is "right" or "clockwise" (Right stick left/right) */
    public static double getRotation()
    {
        if (joystick.getLeftTriggerAxis() > 0.5)
            return joystick.getRightX() / 2;
        return joystick.getRightX();
    }

    /** @return Do we want to shoot? (A) */
    public static boolean doShoot()
    {
        return joystick.getAButtonPressed();
    }

    /** @return Do we want to always run the spinner? Toggles on each press (X) */
    public static boolean toggleSpinner()
    {
        return joystick.getXButtonPressed();
    }

    /** @return Toggle load/don't load (B) */
    public static boolean toggleLoading()
    {
        return joystick.getBButtonPressed();
    }

    /** @return Reverse intake (POV right) */
    public static boolean reverseIntake()
    {
        return joystick.getPOV() == 90;
    }

    /** @return Shift into high gear?  (Left stick push) */
    public static boolean shiftHigh()
    {
        return joystick.getLeftStickButton();
    }

    /** @return Shift into low gear? (Right stick push) */
    public static boolean shiftLow()
    {
        return joystick.getRightStickButton();
    }

    /** @return Toggle climbing arm down/up (right bumper) */
    public static boolean toggleArmAngle()
    {
        return joystick.getRightBumperPressed();
    }

    /** @return Extend arm, positive for 'out' (POV up/down) */
    public static double extendArm()
    {
        if (joystick.getPOV() == 0)
            return 1.0;
        else if (joystick.getPOV() == 180)
            return -1.0;
        return 0.0;
    }

    public static boolean armManualPressed()
    {
        return buttons.getRawButtonPressed(3);
    }

    public static boolean armHighPressed()
    {
        return buttons.getRawButtonPressed(8);
    }

    public static boolean armMidPressed()
    {
        return buttons.getRawButtonPressed(5);
    }

    public static boolean armLowPressed()
    {
        return buttons.getRawButtonPressed(4);
    }
}
