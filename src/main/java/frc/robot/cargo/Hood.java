// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Hood that controls ball ejection angle */
public class Hood extends SubsystemBase
{
    /** Encoder steps per percent movement */
    private final double STEPS_PER_PERC = 2048 * 1;

    /** Position is limited to 0 .. max [percent] */
    private final double MAX_POS_PERC = 500.0;
    
    /** Maximum speed [mm/s] */
    // About half the actual max speed is a good setting
    private final double MAX_PERC_PER_SEC = 50.0;
    
    private final WPI_TalonFX hood = new WPI_TalonFX(RobotMap.HOOD);
    private final DigitalInput home = new DigitalInput(RobotMap.HOOD_HOME);

    private final NetworkTableEntry setpoint = SmartDashboard.getEntry("HoodSetpoint");
    private final NetworkTableEntry position = SmartDashboard.getEntry("Hood");
    private final NetworkTableEntry at_home = SmartDashboard.getEntry("HoodHome");

    public Hood()
    {
        hood.configFactoryDefault();
        hood.clearStickyFaults();
        hood.setNeutralMode(NeutralMode.Brake);
    
        hood.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        // Minimize deadband (0.1 %). Default is 0.04 (4 %)
        hood.configNeutralDeadband(0.001);

        // Use maximum of 10 V to have headroom when battery drops from 12 V
        hood.configVoltageCompSaturation(10.0);
        hood.enableVoltageCompensation(true);
        
        // To tune, set all following gains to zero,
        // then adjust in this order within PhoenixTuner
        hood.selectProfileSlot(0, 0);
        hood.config_kP(0, 0.2);
        // kD ~ 10 * kP
        hood.config_kD(0, 2.0);

        // Find remaining error, set Izone to maybe twice that
        // so integral is zeroed when we are outside of the zone
        hood.config_IntegralZone(0, 1000);
        // Could also configure max integral
        // primary.configMaxIntegralAccumulator(0, 1024);
        // Increase kI to eliminate residual error
        hood.config_kI(0, 0.0);

        // Max speed in encoder ticks per 0.1 seconds;
        final double max_speed = MAX_PERC_PER_SEC * STEPS_PER_PERC * 0.1;
        hood.configMotionCruiseVelocity(max_speed);
        hood.configMotionAcceleration(max_speed);
        // hood.configMotionSCurveStrength(4);

        setpoint.setDefaultDouble(5.0);
    }

    public void reset()
    {
        hood.setSelectedSensorPosition(0);
    }
    
    /** Reset position encoder */
    public void home()
    {
        // TODO
    }

    /** @return Position in mm */
    public double getPosition()
    {
        return hood.getSelectedSensorPosition() / STEPS_PER_PERC;
    }

    public boolean atHome()
    {
        // REV magnetic limit switch contains an internal pull-up resistor,
        // and is 'active low', connecting to ground when detecting the magnet.
        // If we're not at the limit, i.e. magnet is not at the sensor, it reports 'true'
        // because of the pull-up.
        // If we are at the limit, i.e. magnet is close to the sensor, it reports 'false'
        // because of the active low.
        // When we disconnect the cable, the pull-up internal to the RoboRIO reports 'true',
        // so not really fail-safe.
        return ! home.get();
    }

    /** @param output Output -1..1 to motor */
    public void setOutput(final double output)
    {
        hood.set(TalonFXControlMode.PercentOutput, output);
    }

    public void setPosition()
    {
        final double ticks = MathUtil.clamp(setpoint.getDouble(0.0), 0, MAX_POS_PERC) * STEPS_PER_PERC;
        // hood.set(TalonFXControlMode.Position, ticks);
        hood.set(TalonFXControlMode.MotionMagic, ticks);
    }

    @Override
    public void periodic()
    {
        position.setDouble(getPosition());
        at_home.setBoolean(atHome());
    }
} 
