package frc.robot;
import edu.wpi.first.wpilibj.XboxController;

/** Everything related to the 'operator interface'
 * 
 *  Helpers for reading joystick and buttons
 */
public class OperatorInterface
{
    private final static XboxController joystick = new XboxController(0);

    /** @return Speed that driver requests, -1..1, positive is "forward" */
    public static double getSpeed()
    {
        return -joystick.getLeftY();
    }

    /** @return Rotation that driver requests, -1..1, positive is "right" or "clockwise" */
    public static double getRotation()
    {
        return joystick.getRightX();
    }
}
