// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.camera;

import frc.robot.drivetrain.Drivetrain;

/** Command for rotating to target */
public class RotateToTargetCommand extends RotateToCameraCommand
{
    public RotateToTargetCommand(final Drivetrain drivetrain)
    {
        super(drivetrain, "limelight-front", true, false, 1);
        configure(0, 0, 0);
    }
}
