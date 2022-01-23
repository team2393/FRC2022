// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that automatically changes drivetrain gears */
public class AutoShiftCommand extends CommandBase
{
    private final Drivetrain drivetrain;

    public AutoShiftCommand(final Drivetrain drivetrain)
    {
        setName("Auto Shift");
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute()
    {
        // Get average speed of left, right side,
        // i.e., the overall speed of the chassis.
        // We don't care if moving forward or backwards,
        // only absolute value matters
        final double avg_speed = Math.abs((drivetrain.getLeftSpeed() + drivetrain.getRightSpeed()) / 2);
        // Automatically select high gear when we're fast enough,
        // or low gear when too slow.
        // In betweeen, leave it as is.
        // TODO Find good speed thresholds
        if (avg_speed > 1.5)
            drivetrain.shiftgear(true);
        else if (avg_speed < 1.0)
            drivetrain.shiftgear(false);
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.shiftgear(false);
    }
}
