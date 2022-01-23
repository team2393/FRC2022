// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Spinner used to eject balls */
public class Spinner extends SubsystemBase
{
    /** Encoder steps per revolution of spinner wheel */
    // TODO Calibrate on real robot
    private final double STEPS_PER_REV = 2048 * 1;
    
    private final WPI_TalonFX primary = new WPI_TalonFX(RobotMap.PRIMARY_SPINNER);
    private final WPI_TalonFX secondary = new WPI_TalonFX(RobotMap.SECONDARY_SPINNER);
    
    // TODO Find good values!
    // SysId on plywood prototype
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.8287, 0.1148, 0.011757);
    private final PIDController pid = new PIDController(0.14831, 0, 0);

    public Spinner()
    {
        // Primary and secondary motors are on opposite sides of spinner,
        // one needs to be inverted.
        // TODO Invert such that positive speed setting will 'eject' balls
        initializeMotor(primary, false);
        initializeMotor(secondary, true);
        
        // We command the primary motor, secondary follows
        secondary.follow(primary);

        SmartDashboard.setDefaultNumber("SpinnerSetpoint", 10.0);
    }
    
    /** @param motor Motor to initialize
     *  @param invert Should motor direction be inverted?
     */
    private void initializeMotor(final WPI_TalonFX motor, final boolean invert)
    {
        // Motors remember certain settings. We don't know if the motor
        // is fresh out of the box or had been used on a different robot.
        // ==> Make sure that we start with the default configuration.
        motor.configFactoryDefault();
        motor.clearStickyFaults();
        // Spinner tends to, well, spin real fast.
        // When turned off, just allow it to wind down,
        // no need to stop it hard via 'brake' mode.
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setInverted(invert);
    }


    /** Reset position encoder */
    public void reset()
    {
        primary.setSelectedSensorPosition(0);
    }

    /** @return Position in revs */
    public double getPosition()
    {
        // Convert encoder steps into revolutions of spinner wheels
        return primary.getSelectedSensorPosition() / STEPS_PER_REV;
    }

    /** @return Spinner motor voltage */
    public double getVoltage()
    {
        return primary.getMotorOutputVoltage();
    }

    /** @return Measured speed in revs/sec */
    public double getSpeed()
    {
        // Convert encoder steps per 0.1 sec. into revs per second
        return primary.getSelectedSensorVelocity() / STEPS_PER_REV * 10.0;
    }

    /** @param voltage Voltage for spinner motors */
    public void setVoltage(final double voltage)
    {
        primary.setVoltage(voltage);
    }

    /** Run at "SpinnerSetpoint" RPS */
    public void run()
    {
        setSpeed(SmartDashboard.getNumber("SpinnerSetpoint", 0.0));
    }

    /** Stop (allow to run down, no hard brake) */
    public void stop()
    {
        setVoltage(0);
    }

    /** @param speed Desired speed in revs/sec */
    public void setSpeed(final double speed)
    {
        // Use FF and PID to estimate voltage needed for that speed
        double voltage = feedforward.calculate(speed) + pid.calculate(getSpeed(), speed);
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        setVoltage(voltage);
    }
    
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Spinner RPS", getSpeed());
    }
} 
