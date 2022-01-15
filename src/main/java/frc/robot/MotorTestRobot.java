// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Robot for testing motors
 * 
 *  Includes examples for controlling position or speed
 *  via feed forward and/or PID.
 */
public class MotorTestRobot extends TimedRobot
{
    /** Basic encoder steps per rev on Falcon: 2048 */
    private final double STEPS_PER_REV = 2048;

    /** Motor controller */
    private final WPI_TalonFX motor = new WPI_TalonFX(1);

    /** Feed-forward that helps compute the voltage for a desired speed.
     *  See also ElevatorFeedforward, ArmFeedforward
     */
    private final SimpleMotorFeedforward speed_feedforward = new SimpleMotorFeedforward(0.50692, 0.10621, 0.0037104);

    /** PID controller for speed, to be used with speed_feedforward  */
    private final PIDController speed_pid = new PIDController(0, 0, 0);

    /** PID controller for position */
    private final PIDController position_pid = new PIDController(0.1, 0.01, 0);

    /** PID controller for position, using profile
     * 
     *  P, I, D gains should be same as plain position PID.
     *  Max speed of our motor is ~100 rev/sec.
     *  For smooth move example, allow about half that, 50 rev/sec.
     *  Accelerate by 30 rev/s/s,
     *  so after one second speed will be 30 rev/sec,
     *  and we reach 50 rev/sec in about 1.7 sec.
     */
    private final ProfiledPIDController position_profile =
                  new ProfiledPIDController(0.1, 0.01, 0,
                                            new TrapezoidProfile.Constraints(50, 30));

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Motor Test *****");

        // Configure motor
        motor.configFactoryDefault();
        motor.clearStickyFaults();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(false);

        SmartDashboard.putData("Position PID", position_pid);
        SmartDashboard.putData("Speed PID", speed_pid);
    }
    
    public double getPosition()
    {
        // Convert encoder counts into revs
        return motor.getSelectedSensorPosition() / STEPS_PER_REV;
    }
    
    public double getSpeed()
    {
        // Convert encoder counts per 0.1 sec into revs per 1.0 second
        return motor.getSelectedSensorVelocity() / STEPS_PER_REV * 10.0;
    }

    @Override
    public void robotPeriodic()
    {
        // Whatever the mode, display general motor info
        final double voltage = motor.getMotorOutputVoltage();
        final double speed = getSpeed();
        SmartDashboard.putNumber("Voltage", voltage);
        SmartDashboard.putNumber("Revs", getPosition());
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
    public void autonomousInit()
    {
        // Reset profile, required for smooth startup of autoProfiledPositionExample
        position_profile.reset(getPosition());
    }

    @Override
    public void autonomousPeriodic()
    {
        // Pick what to do in auto mode
        autoSpeedExample();
        // autoPositionExample();
        // autoProfiledPositionExample();
    }

    private void autoSpeedExample()
    {
        final double desired_speed;
        // Every 5 seconds, toggle motor between idle and some speed in revs/sec
        if ((System.currentTimeMillis() / 5000) % 2 == 0)
            desired_speed = 50.0;
        else
            desired_speed = 10.0;

        // Compute voltage for the desired speed
        final double actual_speed = getSpeed();

        // Feed-forward does this, maybe with added static and acceleration gain:
        // double kV = 0.1129943502;
        // double voltage = desired_speed * kV;
        // That gets us pretty close to the desired speed if nothing is loading
        // the motor down.
        // PID is added to correct for what we don't know.
        // The 'integral' term is most important to correct the last small difference.
        double voltage = speed_feedforward.calculate(desired_speed);
                      // + speed_pid.calculate(actual_speed, desired_speed);
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        
        motor.setVoltage(voltage);
    }

    private void autoPositionExample()
    {
        final double desired_position;
        // Every 5 seconds, toggle between two positions
        if ((System.currentTimeMillis() / 5000) % 2 == 0)
            desired_position = 100.0;
        else
            desired_position = 0.0;

        // For position, we can't use a feed forward since we can't
        // predict what voltage is needed to reach a certain position.
        // Always depends on the current position, so need PID.
        // PID 'p' mostly gets us to desired position.
        // PID 'i' corrects for the remaining small but constant position error.
        // PID 'd' can dampen the PID.
        // Finding good values is more art than science.
        // Start with small values, then increase until system oscillates.
        final double actual_position = getPosition();
        double voltage = position_pid.calculate(actual_position, desired_position);
        // Voltage must stay within +-12 V range.
        // Limit it a bit further to avoid crazy acceleration when
        // we're far from the setpoint
        voltage = MathUtil.clamp(voltage, -7.0, 7.0);
    
        motor.setVoltage(voltage);
    }

    private void autoProfiledPositionExample()
    {
        final double desired_position;
        // Every 10 seconds, toggle between two positions
        if ((System.currentTimeMillis() / 10000) % 2 == 0)
            desired_position = 100.0;
        else
            desired_position = 0.0;

        // Profiled PID uses the same P, I, D gains as the plain position PID,
        // so is tuned/adjusted the same way.
        // Assume we are at position 0 and want to go to position 100.
        // With plain PID, the setpoint is thus 100, the error is large,
        // such is voltage = P * error, often resulting in a crazy intial acceleration.
        // The profiled PID uses 100 as the 'goal', and internally moves the 'setpoint'
        // from 0 to 100 based on the max. velocity and accelaration that we told it to use.
        // This results in a smooth move, accelerating up from pos. 0, running at max speed for a while,
        // then de-accelrating as we reach 100.
        final double actual_position = getPosition();
        double voltage = position_profile.calculate(actual_position, desired_position);
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
