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
import frc.robot.cargo.BallHandling;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.util.LookupTable;

/** Command for setting spinner speed and rotating robot to hit target */
public class AutoFireCommand extends CommandBase
{
    private final Drivetrain drivetrain;
    private final BallHandling ball_handling;
    final private NetworkTableEntry valid, horiz, vert;

    // Camera data can 'jump around'.
    // Median filter picks the middle value of the last N readings.
    // Compared to an everage of the last N readings, outliers are
    // simply ignored.
    // Making N too large, however, adds a delay to position updates,
    // so use a small N.
    private final MedianFilter median_dist = new MedianFilter(3);
    private final MedianFilter median_error = new MedianFilter(3);

    private final LookupTable speed_for_dist = new LookupTable(
    // Distance [ty], Spinner Speed [rps]
           10.00, 64,
            3.15, 70,
           -0.70, 70,
           -4.50, 76,
           -6.3,  82,
           -6.60, 84,
           -9.65, 90
    );

    private final double rmin = 0.25, rp = 0.02, rmax = 0.35;

    public AutoFireCommand(final Drivetrain drivetrain, final BallHandling ball_handling)
    {
        this.drivetrain = drivetrain;
        this.ball_handling = ball_handling;
        addRequirements(drivetrain);
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
        valid = table.getEntry("tv");
        horiz = table.getEntry("tx");
        vert = table.getEntry("ty");
    }

    @Override
    public void initialize()
    {
        median_dist.reset();
        median_error.reset();
        SmartDashboard.putBoolean("High", true);
    }

    @Override
    public void execute()
    {
        // Check if camera has valid pixel distance
        if (valid.getDouble(0) < 1)
        {
            median_dist.reset();
            median_error.reset();
        }
        else
        {
            // Set spinner speed based on distance
            final double pixel_dist = vert.getDouble(0);

            // Filter
            final double filtered_pixel_dist = median_dist.calculate(pixel_dist);
    
            final double speed = speed_for_dist.lookup(filtered_pixel_dist);
    
            SmartDashboard.putNumber("SpinnerSetpoint", speed);

            // Rotate onto target
            double error = horiz.getDouble(0);
            // Filter
            error = median_error.calculate(error);

            // Error is -30 .. +30 degrees.
            // Positive if target is to the right.
            // Drivetrain rotates to the right (clockwise)
            // for positive values, so same sign on error and rotation
            // Ignore small errors
            error = MathUtil.applyDeadband(error, 1.0);

            // Minimum feed forward and prop gain
            double rotation = Math.signum(error)*rmin + rp*error;
            rotation = MathUtil.clamp(rotation, -rmax, rmax);

            // Drive: Allow manual for/back, rotate based on error
            drivetrain.drive(OperatorInterface.getSpeed(), rotation);

            // Tell driver if we're moving left/right or "done"
            if (error > 0.1)
                OperatorInterface.joystick.setRumble(RumbleType.kRightRumble, 1.0);
            else if (error < -0.1)
                OperatorInterface.joystick.setRumble(RumbleType.kLeftRumble, 1.0);
            else
            {
                OperatorInterface.joystick.setRumble(RumbleType.kLeftRumble, 0.0);
                OperatorInterface.joystick.setRumble(RumbleType.kRightRumble, 0.0);
            }

            final boolean on_target = Math.abs(error) < 0.1;
            if (on_target)
                ball_handling.shoot();
        }
    }
}
