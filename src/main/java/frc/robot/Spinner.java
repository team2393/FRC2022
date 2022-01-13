// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Spinner used to eject balls */
public class Spinner extends SubsystemBase
{
    /** Encoder steps per revolution of spinner wheel */
    // TODO Calibrate!
    private final double STEPS_PER_REV = 2048;
    
    private final WPI_TalonFX primary = new WPI_TalonFX(RobotMap.PRIMARY_SPINNER);
    private final WPI_TalonFX secondary = new WPI_TalonFX(RobotMap.SECONDARY_SPINNER);
    
    // TODO Find good values!
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);
    private final PIDController pid = new PIDController(0, 0, 0);

    public Spinner()
    {
        // TODO Initialize both motors!
        // motor.setNeutralMode(NeutralMode.Coasts);

        // Primary and secondary motors are on opposite sides of spinner,
        // one needs to be inverted.
        // TODO Invert such that positive speed setting will 'eject' balls
        secondary.setInverted(true);

        // We command the primary motor, secondary follows
        secondary.follow(primary); 
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

    /** @return Speed in revs/sec */
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
        SmartDashboard.putNumber("Spinner Rev/sec", getSpeed());
    }
} 
