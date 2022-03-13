// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Robot that tests the spinner using Falcon's built-in speed control loop */
public class SpinnerFalconTestRobot extends TimedRobot
{
    /** Encoder steps per revolution of spinner wheel */
    private final double STEPS_PER_REV = 2048 * 1;
    
    private final WPI_TalonFX primary = new WPI_TalonFX(RobotMap.PRIMARY_SPINNER);
    private final WPI_TalonFX secondary = new WPI_TalonFX(RobotMap.SECONDARY_SPINNER);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Spinner Falcon Test *****");

                // Primary and secondary motors are on opposite sides of spinner,
        // one needs to be inverted.
        // Configured such that positive speed setting will 'eject' balls
        initializeMotor(primary, false);
        initializeMotor(secondary, true);
        
        // We command the primary motor, secondary follows
        secondary.follow(primary);

        // Configure PID etc
        // primary.configClosedloopRamp(0.5);
        primary.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        primary.configNominalOutputForward(0.0);
        primary.configNominalOutputReverse(0.0);
        primary.configPeakOutputForward(1.0);
        primary.configPeakOutputReverse(-1.0);
        primary.configClosedLoopPeakOutput(0, 1.0);
        // Use maximum of 8 V
        primary.configVoltageCompSaturation(8.0);
        primary.enableVoltageCompensation(true);
        // primary.configSupplyCurrentLimit(currLimitCfg);
        
        // To tune, set all gains to zero, then
        // adjust in this order
        
        // kF = 1023 for full output / raw count_per_sec velocity at full output
        primary.config_kF(0, 0.0);
        // kP = desired_percent_output * 1023 / error,
        // or output = error * kP with 1023 for 100% output
        primary.config_kP(0, 0.0);
        // kD ~ 10 * kP
        primary.config_kD(0, 0.0);

        // Find remaining error, set Izone to maybe twice that
        // so integral is zeroed when we are outside of the zone
        // primary.config_IntegralZone(0, 1024);
        // Could also configure max integral
        // primary.configMaxIntegralAccumulator(0, 1024);
        // Increase kI to eliminate residual error
        primary.config_kI(0, 0.0);

        // Reset position
        primary.setSelectedSensorPosition(0);

        SmartDashboard.setDefaultNumber("SpinnerSetpoint", 0.0);
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
        // Select smallest possible deadband (default 0.04 = 4%)
        motor.configNeutralDeadband(0.001);
    }

    /** @return Position in revs */
    public double getPosition()
    {
        // Convert encoder steps into revolutions of spinner wheels
        return primary.getSelectedSensorPosition() / STEPS_PER_REV;
    }
    
    /** @return Measured speed in revs/sec */
    public double getSpeed()
    {
        // Convert encoder steps per 0.1 sec. into revs per second
        return primary.getSelectedSensorVelocity() / STEPS_PER_REV * 10.0;
    }

    @Override
    public void robotPeriodic()
    {
        final double speed = getSpeed();
        SmartDashboard.putNumber("Spinner RPS", speed);
        SmartDashboard.putNumber("Spinner RPM", speed * 60.0);
        SmartDashboard.putNumber("Spinner Rev", getPosition());

        // Send out on each period to get better plots
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void teleopPeriodic()
    {
        // In teleop, manually control spinner through joystick
        primary.set(TalonFXControlMode.PercentOutput, OperatorInterface.getSpeed());
    }

    @Override
    public void autonomousPeriodic()
    {
        // In auto, ask spinner to run at desired speed
        double rps = SmartDashboard.getNumber("SpinnerSetpoint", 0.0);
        
        // Convert revs per seconds into encoder counts per 100ms
        double velo = rps * STEPS_PER_REV * 0.1;
        
        primary.set(TalonFXControlMode.Velocity, velo);
    }
}
