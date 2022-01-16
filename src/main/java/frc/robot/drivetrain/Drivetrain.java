// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.auto.TrajectoryHelper;

/** Motors, encoders, .. of the drive chassis */
public class Drivetrain extends SubsystemBase
{
    /** Track width is distance between left and right wheels in meters */
    private static final double TRACK_WIDTH = 0.85;

    /** Kinematics convert between speed of left/right wheels and speed of robot */
    private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    /** Maximum speed in meters/sec, acceleration in (m/s)/s */        
    public static final TrajectoryConfig trajectory_config = new TrajectoryConfig(2.5, 1.0)
                                                            // .addConstraint(new CentripetalAccelerationConstraint(1.0))
                                                               .setKinematics(kinematics);

    /** Encoder steps per meter of drive chassis movement */
    // Start with "1" so "distance" is in units of encoder steps.
    // Drive about 10 meters, measure the exact distance.
    // Then enter the steps / distance, for example
    // 123434 /  Units.inchesToMeters(400.0)
    // ==> "Distance" is now in units of meters
    //
    // Calibrated for 'smashbot'
    private static final double STEPS_PER_METER = 288123.000000 / Units.inchesToMeters(156.75);

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

    /** Odometry estimates where we are */
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

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

        // Reset everything to zero
        reset();
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

        // Set position to x=0, y=0, heading=0
        final Rotation2d zero = Rotation2d.fromDegrees(0);
        odometry.resetPosition(new Pose2d(0, 0, zero), zero);
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

        // Since we're circumventing the diff_drive, reset its safety timer
        diff_drive.feed();
    }

    /** Create a command that drives along a trajectory
     * 
     *  Trajectory starts at X=0, Y=0 and Heading = 0.
     *  Given list of points must contain entries x, y, h,
     *  i.e., total length of x_y_h array must be a multiple of 3.
     * 
     *  @param reversed Are we driving backwards?
     *  @param x_y_z Sequence of points { X, Y, Heading }
     */
    public CommandBase createTrajectoryCommand(final boolean reversed, final double... x_y_h)
    {
        // Turns waypoints into a trajectory
        final Trajectory trajectory = TrajectoryHelper.createTrajectory(reversed, x_y_h);
        
        // Create command that follows the trajectory
        final Supplier<Pose2d> pose = odometry::getPoseMeters;
        final RamseteController follower = new RamseteController();
        final BiConsumer<Double, Double> output_speeds = this::setSpeeds;
        final CommandBase trajectory_follower = new RamseteCommand(trajectory, pose, follower, kinematics, output_speeds, this);

        // Command to stop
        final CommandBase stop = new InstantCommand(() -> drive(0, 0), this);

        // First follow the trajectory. When done, make sure we stop
        // since we can't be 100% sure how the trajectory plays out...
        return new SequentialCommandGroup(trajectory_follower, stop);
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

        // Update the estimated position
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());

        // Show where we think we are
        final Pose2d pose = odometry.getPoseMeters();
        SmartDashboard.putNumber("Pose X", pose.getX());
        SmartDashboard.putNumber("Pose Y", pose.getY());
        SmartDashboard.putNumber("Pose Head", pose.getRotation().getDegrees());
    }
}
