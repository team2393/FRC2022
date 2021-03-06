// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.camera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;
import frc.robot.drivetrain.Drivetrain;

/** Command for rotating based on limelight information */
public class RotateToCameraCommand extends CommandBase
{
    private final Drivetrain drivetrain;
    private double rmin = 0, kp = 0, rmax = 0;
    private final NetworkTableEntry valid, horiz, deviation;

    // Camera data can 'jump around'.
    // Median filter picks the middle value of the last N readings.
    // Compared to an everage of the last N readings, outliers are
    // simply ignored.
    // Making N too large, however, adds a delay to position updates,
    // so use a small N.
    private final MedianFilter median = new MedianFilter(3);

    /** @param drivetrain Drivetrain to move robot
     *  @param limelight_name Name of limelight (as used in network table)
     *  @param deviation_name Name of deviation in network table
     *  @param led_on Turn LED on?
     *  @param pipeline Pipeline 0..9 to use
     */
    public RotateToCameraCommand(final Drivetrain drivetrain,
                                 final String limelight_name,
                                 final String deviation_name,
                                 final boolean led_on,
                                 final int pipeline)
    {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);

        final NetworkTable table = NetworkTableInstance.getDefault().getTable(limelight_name);
        // 0-3 = default, off, blink, on
        table.getEntry("ledMode").setDouble(led_on ? 3.0 : 1.0);
        // Target 0 or drive mode 1? Manual suggests using 'drive' pipeline,
        // not drive mode to reduce bandwidth
        table.getEntry("camMode").setDouble(0.0);
        table.getEntry("pipeline").setDouble(pipeline);
        valid = table.getEntry("tv");
        horiz = table.getEntry("tx");
        deviation = SmartDashboard.getEntry(deviation_name);
        deviation.setDefaultNumber(0.0);
    }

    /** @param rmin Minimum feed-forward rotation
     *  @param kp Proportional gain
     *  @param rmax Maximum rotation
     */
    public void configure(final double rmin, final double kp, final double rmax)
    {
        this.rmin = rmin;
        this.kp = kp;
        this.rmax = rmax;
    }

    @Override
    public void initialize()
    {
        median.reset();
    }

    @Override
    public void execute()
    {
        // Get direction to target, the 'error', from camera
        double error;
        if (valid.getDouble(0) < 1)
           error = 0.0;
        else
            error = horiz.getDouble(0) - deviation.getDouble(0.0);

        // Filter
        error = median.calculate(error);

        // Error is -30 .. +30 degrees.
        // Positive if target is to the right.
        // Drivetrain rotates to the right (clockwise)
        // for positive values, so same sign on error and rotation
        // Ignore small errors
        if (Math.abs(error) < 1)
            error = 0;

        // Minimum feed forward and prop gain
        double rotation = Math.signum(error)*rmin + kp*error;        
        rotation = MathUtil.clamp(rotation, -rmax, rmax);

        // Drive: Allow manual for/back, rotate based on error
        drivetrain.driveDirect(OperatorInterface.getSpeed(), rotation);

        // Tell driver if we're moving left/right or "done"
        if (error > 0.1)
            OperatorInterface.joystick.setRumble(RumbleType.kLeftRumble, 1.0);
        else if (error < -0.1)
            OperatorInterface.joystick.setRumble(RumbleType.kRightRumble, 1.0);
        else
        {
            OperatorInterface.joystick.setRumble(RumbleType.kLeftRumble, 0.0);
            OperatorInterface.joystick.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }
}
