package frc.robot.cargo;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/** REV color sensor for balls
 * 
 *  Detects both presence and color of ball
 */
public class CargoSensor
{
    /** Threshold for detecting a ball.
     * 
     *  Range is 0..2047, the larger the closer
     */
    private static final int DISTANCE_THRESHOLD = 200;

    /** Sensor, connected to RIO MXP like this:
     * 
     *  GND:  MXP 30
     *  3.3V: MXP 33
     *  SCL:  MXP 32
     *  SDA:  MXP 34
     */
    private final ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kMXP);

    public enum CargoInfo
    {
        NOTHING,
        RED,
        BLUE
    }

    public CargoInfo read()
    {
        // 'distance' value grows(!) when object gets closer
        // Small value means far away or nothing detected
        final int distance = sensor.getProximity();        
        if (distance < DISTANCE_THRESHOLD)
        {
            SmartDashboard.putString("Cargo Sensor", "Nothing");
            return CargoInfo.NOTHING;
        }
        
        // Check color: Is it more red or more blue?
        final Color color = sensor.getColor();
        if (color.red > color.blue)
        {
            SmartDashboard.putString("Cargo Sensor", "Red?");
            return CargoInfo.RED;
        }
        else
        {
            SmartDashboard.putString("Cargo Sensor", "Blue?");
            return CargoInfo.BLUE;
        }
    }
}
