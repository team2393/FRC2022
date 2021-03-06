// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/** Active arm which can extend
 * 
 *  The left and right active arm each have a motor.
 */
public class ActiveArm
{
    /** Motor for extending resp. pulling arm in */
    private WPI_TalonFX extender;

    /** Sensor that detects full retraction */
    private DigitalInput retraction_input;
    private AsynchronousInterrupt retraction_interrupt;


    /** When homing, we stop the motor when is_retracted indicates
     *  that we reached the home position.
     *  But in spite of 'brake' mode on the motor, the gears' inertia
     *  keeps us moving a little further, often beyond the limit switch.
     *  This flag latches the 'at limit' state so we stay stopped. 
     */
    private boolean latch_retracted = false;

    /** Have we homed? */
    private boolean homed = false;

    /** Extender encoder counts per meter */
    private static final double EXTENDER_COUNTS_PER_METER = 164260 / 0.73;

    /** Maximum extension in meters. Set to 0.0 to ignore */
    public static final double MAX_EXTENSION = 0.90;

    /** Suggested voltage for moving extender during homing, must be negative */
    public static final double HOMING_VOLTAGE = -1.5;

    // Could also try sysId tool for "elevator?"
    
    /** Feed-forward voltage to hold robot against gravity.
     *  Should be zero to disable, otherwise NEGATIVE
     *  to pull extension 'in'
     */
    public static final double FF_HOLDING_VOLTAGE = 0.0;

    /** Maximum voltage (positive or negative) used by PID */
    public static double MAX_VOLTAGE = 8.0;
    
    /** PID for extension */
    private final ProfiledPIDController extension_pid = new ProfiledPIDController(100.0, 2.0, 0,
                        // Maximum speed [m/s] and acceleration [m/s/s]:
                        new TrapezoidProfile.Constraints(0.4, 0.4));
    
    /** @param motor_id CAN ID of motor
     *  @param limit_id DIO channel of limit switch
     */
    public ActiveArm(final int motor_id, final int limit_id)
    {
        extender = new WPI_TalonFX(motor_id);
        retraction_input = new DigitalInput(limit_id);
        retraction_interrupt = new AsynchronousInterrupt(retraction_input, this::handleIRQ);
        retraction_interrupt.setInterruptEdges(false, true);

        extender.configFactoryDefault();
        extender.clearStickyFaults();
        extender.setNeutralMode(NeutralMode.Brake);

        // Motor on right arm is inverted wind up onto spool the correct way,
        // left arm is not
        extender.setInverted(motor_id == RobotMap.RIGHT_ARM_EXTENDER);
        extender.configOpenloopRamp(1.0);

        reset();
    }

    private void handleIRQ(Boolean rising_edge, Boolean falling_edge)
    {
        if (falling_edge)
            latch_retracted = true;
    }

    /** Reset to resting position, extender all "in" */
    public void reset()
    {
        extension_pid.reset(getExtension());
        latch_retracted = homed = false;
    }

    /** @return Is arm extension all the way "in", fully retracted? */
    public boolean isRetracted()
    {
        // Reports true when not at limit or broken,
        // false when at limit
        return retraction_input.get() == false;
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
        // Hitting limit switch for the first time?
        if (! homed  &&  latch_retracted)
        {   // Set 'home' location == zero
            extender.setSelectedSensorPosition(0);
            extension_pid.reset(0);
            // Remember that we hit home
            homed = true;
        }
        if (homed || latch_retracted)
        {   // From now on, don't move further down
            // (but gearbox inertia might push us a little further)
            setExtenderVoltage(0);
            return true;
        }
        
        setExtenderVoltage(HOMING_VOLTAGE);
        return false;
    }    

    /** @param voltage Entender voltage, positive for "out" */
    public void setExtenderVoltage(final double voltage)
    {
        if (voltage >= 0)
        {   // Moving out: Check that we stay below max extension
            // (ignore when max. is not configured)
            if (MAX_EXTENSION <= 0.0  ||  getExtension() < MAX_EXTENSION)
                extender.setVoltage(voltage);
            else
                extender.setVoltage(0);
            // Reset retraction latch after moving 'up' at some min. speed
            if (voltage > 0.5)
                latch_retracted = homed = false;
        }
        else
        {   // Moving in: Stop when at limit
            if (latch_retracted  ||  isRetracted())
            {   // Remember that we were at limit even if we run over the switch
                latch_retracted = true;
                extender.setVoltage(0);
            }
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
        
        SmartDashboard.putNumber("Extender Voltage", voltage); 
    }
}
