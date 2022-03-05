// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.CycleDelayFilter;
import frc.robot.util.KeepOnFilter;

/** Spinner used to eject balls */
public class Spinner extends SubsystemBase
{
    /** Encoder steps per revolution of spinner wheel */
    private final double STEPS_PER_REV = 2048 * 1;
    
    private final WPI_TalonFX primary = new WPI_TalonFX(RobotMap.PRIMARY_SPINNER);
    private final WPI_TalonFX secondary = new WPI_TalonFX(RobotMap.SECONDARY_SPINNER);
    
    // Values based on SysId
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.62408, 0.10847, 0.0044311);
    private final PIDController pid = new PIDController(0.09015, 0, 0);

    private final NetworkTableEntry spinner_setpoint = SmartDashboard.getEntry("SpinnerSetpoint");
    private final NetworkTableEntry spinner_rps = SmartDashboard.getEntry("Spinner RPS");

    public Spinner()
    {
        // Primary and secondary motors are on opposite sides of spinner,
        // one needs to be inverted.
        // Configured such that positive speed setting will 'eject' balls
        initializeMotor(primary, false);
        initializeMotor(secondary, true);
        
        // We command the primary motor, secondary follows
        secondary.follow(primary);

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

    /** @return Current drawn by motor */
    public double getCurrent()
    {
        return primary.getStatorCurrent();
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
    private final CycleDelayFilter delay = CycleDelayFilter.forSeconds(0.1);
    private final KeepOnFilter remember_shot = new KeepOnFilter(0.1);

    /** @return Change in motor current */
    private double getCurrentChange()
    {
        // Try highpass and change_filter
        // Try different settings to find good current change threshold
        // for detecting ejected ball
        final double change = highpass.calculate(getCurrent());
        SmartDashboard.putNumber("Spinner Current Change", change);
        return change;
    }
    
    /** @return Does current drop suggest a ball was ejected? */
    public boolean isBallEjected()
    {
        final boolean ejected = delay.compute(remember_shot.compute(Math.abs(getCurrentChange()) > 5.0));
        return ejected; 
    }

    // Speed is controlled by FF and PID,
    // with additional consideration for the initial ramp-up after
    // being stopped.

    enum State
    {
        STOPPED,
        STARTUP,
        RUNNING    
    }

    /** Are we in the initial voltage rampup after being stopped? */
    private State state = State.STOPPED;

    /** Voltage until which we perform the slow initial rampup */
    private static final double INITIAL_RAMPUP_THRESHOLD = 4.0;

    /** Slow voltage rampup from stopped state to full 12V to about 3 second */
    private final SlewRateLimiter startup_slew = new SlewRateLimiter(3.5);

    /** Stop (allow to run down, no hard brake) */
    public void stop()
    {
        setVoltage(0);
        // Prepare to start over with initial rampup
        state = State.STOPPED;
    }

    /** @param speed Desired speed in revs/sec */
    public void setSpeed(final double speed)
    {
        // Use FF and PID to estimate voltage needed for that speed
        double voltage = feedforward.calculate(speed) + pid.calculate(getSpeed(), speed);

        if (state == State.STOPPED)
        {
            startup_slew.reset(0);
            state = State.STARTUP;
        }
        if (state == State.STARTUP)
        {   // While in initial rampup, slow the voltage changes ...
            voltage = startup_slew.calculate(voltage);
            // .. until we reach the threshold voltage
            if (Math.abs(voltage) > INITIAL_RAMPUP_THRESHOLD)
                state = State.RUNNING;
        }

        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        setVoltage(voltage);
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

    @Override
    public void periodic()
    {
        final double speed = getSpeed();
        spinner_rps.setDouble(speed);
        // Remove items as they're no longer necessary
        // SmartDashboard.putNumber("Spinner RPM", speed * 60.0);
        // SmartDashboard.putNumber("Spinner Rev", getPosition());
        // SmartDashboard.putNumber("Spinner Voltage", getVoltage());
        // SmartDashboard.putNumber("Spinner Current", getCurrent());
    }
} 
