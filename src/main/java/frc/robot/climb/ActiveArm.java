// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Active arm which can extend
 * 
 *  The left and right active arm each have a motor.
 */
public class ActiveArm extends SubsystemBase
{
    private WPI_TalonFX extender;
    private DigitalInput is_retracted;
 
    /** Extender encoder counts per meter */
    private static final double EXTENDER_COUNTS_PER_METER = 1.0;
                                                       // =  20000 / Units.inchesToMeters(40.0);

    /** Maximum extension in meters. Set to 0.0 to ignore */
    private static final double MAX_EXTENSION = 0.0;

    /** Suggested voltage for moving extender, must be positive */
    private static final double EXTENDER_VOLTAGE = 2.0;

    // Could also try sysId tool for "elevator?"
    
    /** Feed-forward voltage to hold robot against gravity.
     *  Should be zero to disable, otherise NEGATIVE
     *  to pull extension 'in'
     */
    private final double FF_HOLDING_VOLTAGE = 0.0;

    /** Maximum voltage (positive or negative) used by PID */
    private double MAX_VOLTAGE = 6.0;
    
    /** PID for extension */
    private final ProfiledPIDController extension_pid = new ProfiledPIDController(0, 0, 0,
                        // Maximum speed [m/s] and acceleration [m/s/s]
                        new TrapezoidProfile.Constraints(0.1, 0.1));
    
    /** @param motor_id CAN ID of motor
     *  @param limit_id DIO channel of limit switch, may be -1 to not use switch
     */
    public ActiveArm(final int motor_id, final int limit_id)
    {
        extender = new WPI_TalonFX(motor_id);
        is_retracted = limit_id > 0
                     ? new DigitalInput(limit_id)
                     : null;

        extender.configFactoryDefault();
        extender.clearStickyFaults();
        extender.setNeutralMode(NeutralMode.Brake);
        extender.setInverted(false);
        // TODO Dampen down
        extender.configOpenloopRamp(2.0);

        reset();
    }

    /** Reset to resting position, extender all "in" */
    public void reset()
    {
        extender.setSelectedSensorPosition(0);
    }

    /** @return Is arm extension all the way "in", fully retracted? */
    public boolean isRetracted()
    {
        // If there is no switch, use encoder
        if (is_retracted == null)
            return getExtension() <= 0;

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

    /** @return Extension of arm in meters */
    public double getExtension()
    {
        return extender.getSelectedSensorPosition() / EXTENDER_COUNTS_PER_METER;
    }

    /** Configure PID gains
     *  @param kp Prop gain
     *  @param ki Int. gain
     *  @param kd Diff. gain
     */
    public void configurePID(final double kp, final double ki, final double kd,
                             final double max_voltage)
    {
        extension_pid.setPID(kp, ki, kd);
        MAX_VOLTAGE = max_voltage;
    }

    /** @param desired_extension Desired extension in meters */
    public void setExtension(final double desired_extension)
    {
        // Feed-forward voltage:
        // Are we moving the extension "in", i.e. pulling the robot "up"?
        // Then apply at least the voltage required to hold the robot
        // against gravity
        final double actual = getExtension();
        double voltage = 0;
        if (desired_extension < actual)
            voltage += FF_HOLDING_VOLTAGE;

        // Add PID correction to get to desired extension
        voltage += extension_pid.calculate(actual, desired_extension);
        // Limit voltage
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        setExtenderVoltage(voltage);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm Extension", getExtension());
        SmartDashboard.putBoolean("Arm Retracted", isRetracted());
    }
}
