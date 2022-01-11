// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Robot for testing motors */
public class MotorTestRobot extends TimedRobot
{
    // Basic encoder steps per rev on Falcon: 2048
    private final double STEPS_PER_REV = 2048;
    private final WPI_TalonFX motor = new WPI_TalonFX(1);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.17, 0.112994, 0);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Motor Test *****");

        // Configure motor
        motor.configFactoryDefault();
        motor.clearStickyFaults();
    }
    
    @Override
    public void robotPeriodic()
    {
        final double voltage = motor.getMotorOutputVoltage();
        // Convert encoder counts into revs
        final double pos = motor.getSelectedSensorPosition() / STEPS_PER_REV;
        // Convert encoder counts per 0.1 sec into revs per 1.0 second
        final double speed = motor.getSelectedSensorVelocity() / STEPS_PER_REV * 10.0;
        
        SmartDashboard.putNumber("Voltage", voltage);
        SmartDashboard.putNumber("Revs", pos);
        SmartDashboard.putNumber("Revs per sec", speed);
        if (voltage == 0.0)
            SmartDashboard.putNumber("kV", 0.0);
        else
            SmartDashboard.putNumber("kV", Math.abs(voltage/speed));
    }        

    @Override
    public void teleopInit()
    {
        // Reset motor position
        motor.setSelectedSensorPosition(0.0);
    }

    @Override
    public void teleopPeriodic()
    {
        // Set voltage to -12..12 based on 'speed' command from joystick
        final double voltage = 12.0 * OperatorInterface.getSpeed();
        motor.setVoltage(voltage);
    }

    @Override
    public void autonomousPeriodic()
    {
        final double setpoint;
        // Every 5 seconds, toggle motor between idle and some speed in revs/sec
        if ((System.currentTimeMillis() / 5000) % 2 == 0)
            setpoint = 50.0;
        else
            setpoint = 10.0;

        // Compute voltage for the desired speed
        final double speed = motor.getSelectedSensorVelocity() / STEPS_PER_REV * 10.0;
        // TODO: Use feed forward and/or PID
        // PIDController.
        // Later ProfiledPIDController when controlling position

        // double kV = 0.1129943502;
        // double voltage = setpoint * kV;
        double voltage = feedforward.calculate(setpoint);
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    
        motor.setVoltage(voltage);
    }

    /** Called while in simulation */
    @Override
    public void simulationPeriodic()
    {
        // Simulate motor with kV = 10
        double pos = motor.getSelectedSensorPosition();
        pos += motor.getMotorOutputVoltage() * 10.0 * STEPS_PER_REV * TimedRobot.kDefaultPeriod;
        motor.setSelectedSensorPosition(pos);
        // There's no 'setSelectedSensorVelocity()' to simulate motor speed...
    }
}
