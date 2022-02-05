// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Support arm which can extend and rotate */
public class Arm extends SubsystemBase
{
    private Solenoid rotator = new Solenoid(RobotMap.PCM_TYPE, RobotMap.ARM_ROTATOR);
    private WPI_TalonFX extender = new WPI_TalonFX(RobotMap.ARM_EXTENTER);
    private DigitalInput is_retracted = new DigitalInput(RobotMap.ARM_EXTENDER_AT_IN_LIMIT);
 
    /** Extender encoder counts per meter */
    private static final double EXTENDER_COUNTS_PER_METER = 1.0;
                                                       // =  20000 / Units.inchesToMeters(40.0);

    /** Maximum extension in meters. Set to 0.0 to ignore */
    private static final double MAX_EXTENSION = 0.0;

    /** Suggested voltage for moving extender, must be positive */
    private static final double EXTENDER_VOLTAGE = 2.0;

    // Could also try sysId tool for "arm",
    // then use PID for rotator speed and profiledPID for angle 

    public Arm()
    {
        extender.configFactoryDefault();
        extender.clearStickyFaults();
        extender.setNeutralMode(NeutralMode.Brake);
        extender.setInverted(false);

        reset();
    }

    /** Reset to resting position:
     *  Extender all "in", rotator "down" without power
     */
    public void reset()
    {
        extender.setSelectedSensorPosition(0);
        rotator.set(false);
    }

    /** @return Is arm extension all the way "in", fully retracted? */
    public boolean isRetracted()
    {
        // Sensor should be fail-safe:
        // Reports true when not at limit,
        // false when at limit or broken
        return is_retracted.get() == false;
    }

    /** Perform homing operation
     * 
     *  Move extender "in" until it hits the limit,
     *  then reset encoder to zero.
     * 
     *  @return Are we done, fully retracted?
     */
    public boolean homing()
    {
        if (isRetracted())
        {
            setExtenderVoltage(0);
            extender.setSelectedSensorPosition(0);
            return true;
        }
        
        setExtenderVoltage(-EXTENDER_VOLTAGE);
        return false;
    }    

    /** @param down Rotate arm down? Otherwise up. */
    public void setAngle(final boolean down)
    {
        rotator.set(down);
    }

    /** @return Is arm down? */
    public boolean getAngle()
    {
        return rotator.get();
    }

    /** @param voltage Entender voltage, positive for "out" */
    // need to add a solenoid for safety arm
    public void setExtenderVoltage(final double voltage)
    {
        if (voltage >= 0)
        {   // Moving out: Check that we stay below max extension
            // (ignore when  max. is not configured)
            if (MAX_EXTENSION <= 0.0  ||  getExtension() < MAX_EXTENSION)
                extender.setVoltage(voltage);
            else
                extender.setVoltage(0);
        }
        else
        {   // Moving in: Stop when at limit
            if (isRetracted())
                extender.setVoltage(0);
            else
                extender.setVoltage(voltage);
        }
    }

    /** @return Extension of arm in meters */
    public double getExtension()
    {
        return extender.getSelectedSensorPosition() / EXTENDER_COUNTS_PER_METER;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Arm Down", rotator.get());
        SmartDashboard.putNumber("Arm Extension", getExtension());
        SmartDashboard.putBoolean("Arm Retracted", isRetracted());
    }
}
