// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
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
    private DigitalInput is_retracted = new DigitalInput(RobotMap.ARM_EXTENDER_AT_IN_LIMIT);
 
    /** Rotator encoder counts per degree */
    private static final double ANGLE_COUNTS_PER_DEGREE = 1.0;

    /** Extender encoder counts per meter */
    private static final double EXTENDER_COUNTS_PER_METER = 1.0;
                                                       // =  20000 / Units.inchesToMeters(40.0);

    /** Maximum extension in meters. Set to 0.0 to ignore */
    private static final double MAX_EXTENSION = 0.0;

    /** Suggested voltage for moving extender, must be positive */
    private static final double EXTENDER_VOLTAGE = 2.0;

    // Arm control is PID with feed forward based on gravity * cos(angle),
    // https://trickingrockstothink.com/blog_posts/2019/10/26/controls_supp_arm.html
    //
    // Try adding some static gain,
    // and note that the center of gravity and thus the torque required
    // to hold the arm changes when extended in/out

    /** Rotator static gain, minimum voltage to always use when moving */
    private static final double ANGLE_STATIC_GAIN = 0.0;

    /** Rotator gravity gain when extender is all the way "in" */
    private static final double ANGLE_GRAVITY_GAIN_IN = 0.0;

    /** Rotator gravity gain when extender is all the way "out" */
    private static final double ANGLE_GRAVITY_GAIN_OUT = 0.0;

    /** PID controller for rotator angle */
    private final PIDController angle_pid = new PIDController(0, 0, 0);

    /** Maximum voltage used to dampen rotator angle control */
    private double MAX_ARM_VOLTAGE = 6.0;

    // Could also try sysId tool for "arm",
    // then use PID for rotator speed and profiledPID for angle 

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
        {   // Moving in: Stop when at limit
            if (isRetracted())
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
    public void configurePID(final double kp, final double ki, final double kd, final double max_voltage)
    {
        angle_pid.setPID(kp, ki, kd);
        MAX_ARM_VOLTAGE = max_voltage;
    }

    /** @param desired_degrees Angle to which arm should be moved */
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
        final double extension_factor = getExtension() / MAX_EXTENSION;
        final double gravity_gain = ANGLE_GRAVITY_GAIN_IN  +  extension_factor * (ANGLE_GRAVITY_GAIN_OUT - ANGLE_GRAVITY_GAIN_IN);
        // cos( 0 deg) = 1, so near zero degrees we apply the full voltage to fight gravity
        // cos(90 deg) = 0, so when fully vertical we don't need any voltage.
        // In between, cos(angle) knows what to do, but note that it needs radians, not degrees.
        voltage += gravity_gain * Math.cos(Math.toRadians(angle));

        // Add PID to the feed-forward voltage
        voltage += angle_pid.calculate(angle, desired_degrees);

        // Keep within bounds
        voltage = MathUtil.clamp(voltage, -MAX_ARM_VOLTAGE, MAX_ARM_VOLTAGE);

        rotator.setNeutralMode(NeutralMode.Brake);
        setRotatorVoltage(voltage);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Arm Extension", getExtension());
        SmartDashboard.putBoolean("Arm Retracted", isRetracted());
    }
}
