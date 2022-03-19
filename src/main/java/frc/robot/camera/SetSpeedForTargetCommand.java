// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.camera;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.LookupTable;

/** Command for setting spinner speed to hit target */
public class SetSpeedForTargetCommand extends CommandBase
{
    final private NetworkTableEntry valid, vert;

    // Camera data can 'jump around'.
    // Median filter picks the middle value of the last N readings.
    // Compared to an everage of the last N readings, outliers are
    // simply ignored.
    // Making N too large, however, adds a delay to position updates,
    // so use a small N.
    private final MedianFilter median = new MedianFilter(3);

    private final LookupTable speed_for_dist = new SpeedLookupTable();

    public SetSpeedForTargetCommand(final String limelight_name)
    {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
        valid = table.getEntry("tv");
        vert = table.getEntry("ty");
    }

    @Override
    public void initialize()
    {
        median.reset();
        SmartDashboard.putBoolean("High", true);
    }

    @Override
    public void execute()
    {
        // Check if camera has valid pixel distance
        if (valid.getDouble(0) < 1)
        {
            median.reset();
            // TODO What to show? NaN?
            SmartDashboard.putNumber("Raw Dist", 100);
            SmartDashboard.putNumber("Filtered Dist", 100);
        }
        else
        {
            final double pixel_dist = vert.getDouble(0);

            // Filter
            final double filtered_pixel_dist = median.calculate(pixel_dist);
    
            final double speed = speed_for_dist.lookup(filtered_pixel_dist);
    
            SmartDashboard.putNumber("Raw Dist", pixel_dist);
            SmartDashboard.putNumber("Filtered Dist", filtered_pixel_dist);
            SmartDashboard.putNumber("SpinnerSetpoint", speed);
        }
    }
}
