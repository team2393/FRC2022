// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Support arm which can extend and rotate */
public class Arm extends SubsystemBase
{
    private WPI_TalonFX rotator = new WPI_TalonFX(RobotMap.ARM_ROTATOR);
    private WPI_TalonFX extender = new WPI_TalonFX(RobotMap.ARM_EXTENTER);
    private DigitalInput in_limit = new DigitalInput(RobotMap.ARM_EXTENDER_AT_IN_LIMIT);
 
    /** Rotator encoder counts per degree */
    private static final double ANGLE_COUNTS_PER_DEGREE = 1.0;

    /** Extender encoder counts per meter */
    private static final double EXTENDER_COUNTS_PER_METER = 1.0;
    // =  20000 / Units.inchesToMeters(40.0);

    /** Maximum extension in meters. Set to 0.0 to disable */
    private static final double MAX_EXTENSION = 0.0;

    /** Suggested voltage for moving extender, must be positive */
    private static final double EXTENDER_VOLTAGE = 2.0;

    /** Static rotator gain, minimum voltage to always use when moving */
    private static final double ANGLE_STATIC_GAIN = 0.0;

    /** Gravity rotator gain when extender is all the way "in" */
    private static final double ANGLE_GRAVITY_GAIN_IN = 0.0;

    /** Gravity rotator gain when extender is all the way "out" */
    private static final double ANGLE_GRAVITY_GAIN_OUT = 0.0;

    private final PIDController angle_pid = new PIDController(0, 0, 0);

    // TODO Try sysId tool for "arm",
    // then use PID for rotator speed, profiledPID for angle 

    public Arm()
    {
        extender.configFactoryDefault();
        extender.clearStickyFaults();
        extender.setNeutralMode(NeutralMode.Brake);
        extender.setInverted(false);

        rotator.configFactoryDefault();
        rotator.clearStickyFaults();
        rotator.setInverted(false);

        reset();
    }

    /** Reset to resting position:
     *  Extender all "in", rotator "down" without power
     */
    public void reset()
    {
        extender.setSelectedSensorPosition(0);

        rotator.setSelectedSensorPosition(0);
        rotator.setNeutralMode(NeutralMode.Coast);
    }

    /** Perform homing operation
     * 
     *  Move extender "in" until it hits the limit,
     *  and then resets the encoder to zero.
     * 
     *  @return Are we done, at the limit?
     */
    public boolean homing()
    {
        setExtenderVoltage(-EXTENDER_VOLTAGE);
        if (in_limit.get())
        {
            extender.setSelectedSensorPosition(0);
            return true;
        }
        return false;
    }    

    /** @param voltage Rotator voltage, positive for "up" */
    public void setRotatorVoltage(final double voltage)
    {
        rotator.setVoltage(voltage);
    }

    /** @param voltage Entender voltage, positive for "out" */
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
        {
            // Moving in: Stop when at limit
            if (in_limit.get())
                extender.setVoltage(0);
            else
                extender.setVoltage(voltage);
        }
    }

    /** @return Angle of rotator.
     *           0 degrees = flat, down, horizontal
     *          90 degree = all up, vertical
     */
    public double getAngle()
    {
        return rotator.getSelectedSensorPosition() / ANGLE_COUNTS_PER_DEGREE;
    }

    /** @return Extension of arm in meters */
    public double getExtension()
    {
        return extender.getSelectedSensorPosition() / EXTENDER_COUNTS_PER_METER;
    }

    /** Configure angle PID gains */
    public void configurePID(final double kp, final double ki, final double kd)
    {
        angle_pid.setPID(kp, ki, kd);
    }

    public void setAngle(final double desired_degrees)
    {
        // If we're very close to zero, and that's where we want to be,
        // just turn motor off and let it settle down on its own
        final double angle = getAngle();
        final double error = desired_degrees - angle;        
        if (desired_degrees < 5  &&  Math.abs(error) < 5)
        {
            setRotatorVoltage(0);
            rotator.setNeutralMode(NeutralMode.Coast);
            return;
        }
        
        // Are we above or below the desired angle?
        // Apply the minimum get-going voltage
        double voltage = ANGLE_STATIC_GAIN * Math.signum(error);

        // Add the voltage required to counteract gravity.
        // Interpolate between ANGLE_GRAVITY_GAIN_IN when all "in"
        // and ANGLE_GRAVITY_GAIN_OUT when all "out"
        final double extension = getExtension() / MAX_EXTENSION;
        final double grav_gain = ANGLE_GRAVITY_GAIN_IN + extension * (ANGLE_GRAVITY_GAIN_OUT - ANGLE_GRAVITY_GAIN_IN);
        // cos(0) = 1, so near zero degrees we apply the full voltage to fight gravity
        // cos(op deg) = 0, so when fully vertical we don't need any voltage.
        // In between, cos(angle) knows what to do, but note that it needs radians, not degrees.
        voltage += grav_gain * Math.cos(Math.toRadians(angle));

        // Add PID to the feed-forward voltage
        voltage += angle_pid.calculate(angle, desired_degrees);

        rotator.setNeutralMode(NeutralMode.Brake);
        setRotatorVoltage(voltage);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Arm Extension", getExtension());
        SmartDashboard.putBoolean("Arm In", in_limit.get());
    }
}
