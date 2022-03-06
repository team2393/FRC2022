// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.camera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;
import frc.robot.drivetrain.Drivetrain;

/** Command for rotating based on limelight information */
public class RotateToCameraCommand extends CommandBase
{
    private final Drivetrain drivetrain;
    private final PIDController pid = new PIDController(0, 0, 0);
    private final NetworkTableEntry valid, horiz;

    // Camera data can 'jump around'.
    // Median filter picks the middle value of the last N readings.
    // Compared to an everage of the last N readings, outliers are
    // simply ignored.
    // Making N too large, however, adds a delay to position updates,
    // so use a small N.
    private final MedianFilter median = new MedianFilter(3);

    /** @param drivetrain Drivetrain to move robot
     *  @param limelight_name Name of limelight (as used in network table)
     *  @param led_on Turn LED on?
     *  @param drive_mode Drive mode? Otherwise target mode
     *  @param pipeline Pipeline 0..9 to use
     */
    public RotateToCameraCommand(final Drivetrain drivetrain,
                                 final String limelight_name,
                                 final boolean led_on,
                                 final boolean drive_mode,
                                 final int pipeline)
    {
        this.drivetrain = drivetrain;

        final NetworkTable table = NetworkTableInstance.getDefault().getTable(limelight_name);
        // 0-3 = default, off, blink, on
        table.getEntry("ledMode").setDouble(led_on ? 3.0 : 1.0);
        // Target or drive mode?
        table.getEntry("camMode").setDouble(drive_mode ? 1.0 : 0.0);
        table.getEntry("pipeline").setDouble(pipeline);
        valid = table.getEntry("tv");
        horiz = table.getEntry("tx");
    }

    public void configure(final double kp, final double ki, final double kd)
    {
        pid.setPID(kp, ki, kd);
    }

    @Override
    public void initialize()
    {
        median.reset();
    }

    @Override
    public void execute()
    {
        // Get direction to target, the 'measurement', from camera
        double measurement;
        if (valid.getDouble(0) < 1)
            measurement = 0.0;
        else
            measurement = horiz.getDouble(0);

        // Filter
        measurement = median.calculate(measurement);

        // We want point onto the target
        final double setpoint = 0.0;
        // Use PID to compute the necessary rotation
        double rotation = pid.calculate(measurement, setpoint);
        // rotation += OperatorInterface.getRotation();
        rotation = MathUtil.clamp(rotation, -0.5, 0.5);

        // Drive: Positive rotation is "right", clockwise
        drivetrain.drive(OperatorInterface.getSpeed(), rotation);
    }
}
