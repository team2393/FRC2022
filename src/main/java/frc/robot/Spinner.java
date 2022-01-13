// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Spinner used to eject balls */
public class Spinner extends SubsystemBase
{
    private final double STEPS_PER_REV = 2048;
    private final WPI_TalonFX primary = new WPI_TalonFX(RobotMap.PRIMARY_SPINNER);
    private final WPI_TalonFX secondary = new WPI_TalonFX(RobotMap.SECONDARY_SPINNER);
    
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);
    private final PIDController pid = new PIDController(0, 0, 0);

    public Spinner()
    {
        // TODO Initialize both motors!

        // Primary and secondary motors are on opposite sites of spinner,
        // one needs to be inverted.
        // TODO Invert such that positive speed setting will 'eject' balls
        secondary.setInverted(true); 
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
        return primary.getSelectedSensorPosition() / STEPS_PER_REV;
    }

    /** @return Speed in revs/sec */
    public double getSpeed()
    {
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
        // TODO Use FF and PID
        // double voltage = feedforward.calculate(speed) + pid.calculate(getSpeed(), speed);
        // setVoltage(voltage);
    }
    
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Spinner Rev/sec", getSpeed());
    }
} 
