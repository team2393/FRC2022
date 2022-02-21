// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that tells drivetrain to stop */
public class StayPutCommand extends CommandBase
{
    private final Drivetrain drivetrain;

    public StayPutCommand(final Drivetrain drivetrain)
    {
        setName("Stay Put");
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute()
    {
        drivetrain.drive(0, 0);
    }
}
