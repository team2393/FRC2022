package frc.robot.drivetrain;

// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;

/** Command that drives based on joystick input */
public class DriveByJoystickCommand extends CommandBase
{
    private final Drivetrain drivetrain;

    public DriveByJoystickCommand(final Drivetrain drivetrain)
    {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute()
    {
        drivetrain.drive(OperatorInterface.getSpeed(), OperatorInterface.getRotation());
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.drive(0, 0);
    }
}
