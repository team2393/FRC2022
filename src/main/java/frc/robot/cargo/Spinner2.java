// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.CycleDelayFilter;
import frc.robot.util.KeepOnFilter;

/** Spinner used to eject balls */
public class Spinner2 extends SubsystemBase
{
    /** Encoder steps per revolution of spinner wheel */
    private final double STEPS_PER_REV = 2048 * 1;
    
    private final WPI_TalonFX primary = new WPI_TalonFX(RobotMap.PRIMARY_SPINNER);
    // TODO private final WPI_TalonFX secondary = new WPI_TalonFX(RobotMap.SECONDARY_SPINNER);
    
    private final NetworkTableEntry spinner_setpoint = SmartDashboard.getEntry("SpinnerSetpoint");
    private final NetworkTableEntry spinner_rps = SmartDashboard.getEntry("Spinner RPS");

    public Spinner2()
    {
        // Primary and secondary motors are on opposite sides of spinner,
        // one needs to be inverted.
        // Configured such that positive speed setting will 'eject' balls
        initializeMotor(primary, false);
    // TODO    initializeMotor(secondary, true);
        
        // We command the primary motor, secondary follows
    // TODO    secondary.follow(primary);

        // TODO Configure PID etc
        primary.configClosedloopRamp(2.0);
        primary.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        primary.configNominalOutputForward(0.0);
        primary.configNominalOutputReverse(0.0);
        primary.configPeakOutputForward(1.0);
        primary.configPeakOutputReverse(-1.0);
        primary.configClosedLoopPeakOutput(0, 1.0);
        // Use maximum of 10 V to have headroom when battery drops from 12 V
        primary.configVoltageCompSaturation(10.0);
        primary.enableVoltageCompensation(true);
        
        // To tune, set all following gains to zero,
        // then adjust in this order within PhoenixTuner
        
        // kF = 1023 for full output / raw count_per_sec velocity at full output
        
        // output 0.67 -> 71 rps = 71*2048*0.1 units/sec ~ 14625 units/100ms 
        // -> kF = 1023 * 0.67 / 14625 = 0.047
        primary.config_kF(0, 0.0557);
        // kP = desired_percent_output * 1023 / error,
        // or output = error * kP with 1023 for 100% output
        primary.config_kP(0, 0.04);
        // kD ~ 10 * kP
        primary.config_kD(0, 0.4);

        // Find remaining error, set Izone to maybe twice that
        // so integral is zeroed when we are outside of the zone
        primary.config_IntegralZone(0, 1000);
        // Could also configure max integral
        // primary.configMaxIntegralAccumulator(0, 1024);
        // Increase kI to eliminate residual error
        primary.config_kI(0, 5e-06);

        reset();

        spinner_setpoint.setDefaultDouble(60.0);
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

    /** @return Does current drop suggest a ball was ejected? */
    public boolean isBallEjected()
    {
        return ejected; 
    }

    /** Stop (allow to run down, no hard brake) */
    public void stop()
    {
        setVoltage(0);
    }

    /** @param rps Desired speed in revs/sec */
    public void setSpeed(final double rps)
    {
        // Convert revs per seconds into encoder counts per 100ms
        final double velo = rps * STEPS_PER_REV * 0.1;        
        primary.set(TalonFXControlMode.Velocity, velo);
    }

    public double getSetpoint()
    {
        return spinner_setpoint.getDouble(10.0);
    }
    
    /** Run at "SpinnerSetpoint" RPS */
    public void run()
    {
        setSpeed(getSetpoint());
    }

    // High pass filter shows change in value, any change slower than 0.1 seconds are ignored.
    // This filters out small changes, but rapid drop in current as ball is ejected
    // gets detected.
    private final LinearFilter highpass = LinearFilter.highPass(0.1, 0.02);

    // Remember that we saw a ball get ejected for a little while
    // .. but not too long, because otherwise a "toggle spinner"
    // that was recently issued to start the spinner will be mistaken
    // for an already ejected ball.
    // Delay a little to keep feeder running until ball is out.
    private final CycleDelayFilter delay = CycleDelayFilter.forSeconds(0.05);
    private final KeepOnFilter remember_shot = new KeepOnFilter(0.05);

    private boolean ejected = false;

    @Override
    public void periodic()
    {
        final double current  = primary.getStatorCurrent();
        final double change = highpass.calculate(current);
        // SmartDashboard.putNumber("Spinner Current Change", change);
        ejected = delay.compute(remember_shot.compute(Math.abs(change) > 10.0));

        spinner_rps.setDouble(getSpeed());
    }
} 
