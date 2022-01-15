// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Motors, encoders, .. of the drive chassis */
public class Drivetrain extends SubsystemBase
{
    // Start with "1" so "distance" is in units of encoder steps.
    // Drive about 10 meters, measure the exact distance.
    // Then enter the steps / distance, for example
    // 123434 /  Units.inchesToMeters(400.0)
    // ==> "Distance" is now in units of meters

    // Calibrated for 'smashbot'
    /** Encoder steps per meter of drive chassis movement */
    private final double STEPS_PER_METER = 288123.000000 / Units.inchesToMeters(156.75);

    /** Drive motors */
    private final WPI_TalonFX primary_left    = new WPI_TalonFX(RobotMap.PRIMARY_LEFT_DRIVE),
                              secondary_left  = new WPI_TalonFX(RobotMap.SECONDARY_LEFT_DRIVE),
                              primary_right   = new WPI_TalonFX(RobotMap.PRIMARY_RIGHT_DRIVE),
                              secondary_right = new WPI_TalonFX(RobotMap.SECONDARY_RIGHT_DRIVE);

    /** Differential drive helper for turning forward/backwards and rotation into motor voltages */
    private final DifferentialDrive diff_drive = new DifferentialDrive(primary_left, primary_right);

    // TODO Tune FF settings, then PID settings

    /** FF for motor speed from SysId */
    private final SimpleMotorFeedforward speed_feedforward = new SimpleMotorFeedforward(0.54799, 3.8445, 0.19083);

    /** PIDs for motor speed
     * 
     *  Both the left and right motors behave the same,
     *  so we can use the same feed forward and PID settings.
     *  
     *  While we can actually use the exact same 'speed_feedforward',
     *  we cannot use the same PIDController instance because a PIDController
     *  keeps track of the integral error and the last error, which can differ
     *  for the two sides.
     * 
     *  SysId suggests 0, 3.5927, 0
     */
    private final PIDController left_speed_pid = new PIDController(0, 0, 0),
                                right_speed_pid = new PIDController(0, 0, 0);

    /** Heading and tilt angle sensor */
    private final PigeonIMU pigeon = new PigeonIMU(0);

    public Drivetrain()
    {
        // steps/rev / (steps/meter)
        // steps * 1/rev * meters * 1/steps
        // meters / rev
        // 0.028300446684228608
        double meters_per_rot = 2048 / STEPS_PER_METER;
        System.out.println("Drivetrain meters per rev: " + meters_per_rot);
        // Motors on right need to be inverted
        initializeMotor(primary_left,   false);
        initializeMotor(secondary_left, false);
        initializeMotor(primary_right,   true);
        initializeMotor(secondary_right, true);

        // Have secondaries follow the primaries
        secondary_left.follow(primary_left);
        secondary_right.follow(primary_right);

        // Reset pigeon
        pigeon.configFactoryDefault();
        pigeon.clearStickyFaults();
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
        // When we set zero voltage, brake!
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(invert);
    }

    /** Reset encoders so position is back to "0 meters" */
    public void reset()
    {
        primary_left.setSelectedSensorPosition(0.0);
        primary_right.setSelectedSensorPosition(0.0);

        left_speed_pid.reset();
        right_speed_pid.reset();

        // Reset gyro angle
        pigeon.setFusedHeading(0.0);
        pigeon.setYaw(0.0);
    }

    /** @param kp Proportional gain
     *  @param ki Integral gain
     *  @param kd derivative gain
     */
    public void configurePID(final double kp, final double ki, final double kd)
    {
        left_speed_pid.setPID(kp, ki, kd);
        right_speed_pid.setPID(kp, ki, kd);
    }

    /** @return Distance travelled by left side motor(s) in meters */
    public double getLeftDistance()
    {
        return primary_left.getSelectedSensorPosition() / STEPS_PER_METER; 
    }

    /** @return Distance travelled by right side motor(s) in meters */
    public double getRightDistance()
    {
        return primary_right.getSelectedSensorPosition() / STEPS_PER_METER; 
    }

    /** @return Speed of left side motor(s) in meters/sec */
    public double getLeftSpeed()
    {
        return primary_left.getSelectedSensorVelocity() / STEPS_PER_METER * 10.0; 
    }

    /** @return Speed of right side motor(s) in meters/sec */
    public double getRightSpeed()
    {
        return primary_right.getSelectedSensorVelocity() / STEPS_PER_METER * 10.0; 
    }

    /** @return Voltage of left side motor(s) */
    public double getLeftVoltage()
    {
        return primary_left.getMotorOutputVoltage(); 
    }

    /** @return Voltage of right side motor(s) */
    public double getRightVoltage()
    {
        return primary_right.getMotorOutputVoltage(); 
    }

    /** @return Heading angle in degrees, increasing counterclockwise  */
    public double getHeading()
    {
        return pigeon.getFusedHeading(); 
    }

    /** @param speed -1..1 speed of going back/for. Forward is positive
     *  @param rotation -1..1 speed of rotation. Positive is "right", clockwise
     */
    public void drive(final double speed, final double rotation)
    {
        diff_drive.arcadeDrive(speed, rotation);
    }

    /** @param left_speed Speed in meters/sec for left side
     *  @param right_speed .. and right side
     */
    public void setSpeeds(final double left_speed, final double right_speed)
    {
        double actual_speed = getLeftSpeed();
        primary_left.setVoltage(speed_feedforward.calculate(left_speed)   + left_speed_pid.calculate(actual_speed, left_speed));

        actual_speed = getRightSpeed();
        primary_right.setVoltage(speed_feedforward.calculate(right_speed) + right_speed_pid.calculate(actual_speed, right_speed));

        // Since we're circumventing the drive train, reset its safety timer
        diff_drive.feed();
    }

    /** Create a command that runs the drve train along a trajectory
     * 
     *  Trajectory starts at X=0, Y=0 and Heading = 0.
     *  Given list of points must contain entries x, y, h,
     *  i.e., total length of x_y_h array must be a multiple of 3.
     * 
     *  @param x_y_z Sequence of points { X, Y, Heading }
     */
    public CommandBase createTrajectoryCommand(final double... x_y_h)
    {
        if (x_y_h.length % 3 != 0)
            throw new IllegalArgumentException("List of { X, Y, Heading } contains " + x_y_h.length + " entries?!");
        
        // TODO Create trajectory, return RamseteCommand to follow,
        //      instead of just printing what it should do
        final SequentialCommandGroup result = new SequentialCommandGroup();
        result.addCommands(new PrintCommand("Start at 0, 0, 0"));
        for (int i=0; i<x_y_h.length; i += 3)
            result.addCommands(new PrintCommand("Move to " + x_y_h[i] + ", " + x_y_h[i+1] + ", " + x_y_h[i+2]));
        result.addCommands(new PrintCommand("Stop"));

        result.addRequirements(this);

        return result;
    }

    @Override
    public void periodic()
    {
        // Display generally useful drivetrain info
        double avg = (getLeftDistance() + getRightDistance()) / 2;
        SmartDashboard.putNumber("Distance",  avg);

        avg = (getLeftSpeed() + getRightSpeed()) / 2;
        SmartDashboard.putNumber("Speed",  avg);

        SmartDashboard.putNumber("Heading", pigeon.getFusedHeading());
    }
}
