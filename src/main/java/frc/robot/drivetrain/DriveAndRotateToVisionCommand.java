package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;
import frc.robot.camera.GuessingUDPClient;

/** Command that rotates based on vision info */
public class DriveAndRotateToVisionCommand extends CommandBase
{
    private final Drivetrain drivetrain;
    private final GuessingUDPClient guess;

    public DriveAndRotateToVisionCommand(final Drivetrain drivetrain, final GuessingUDPClient guess)
    {
        this.drivetrain = drivetrain;
        this.guess = guess;
        addRequirements(drivetrain);

        SmartDashboard.setDefaultNumber("VisRotGain", 0.0);
    }

    @Override
    public void execute()
    {
        // Start with rotation from joystick
        double rotation = OperatorInterface.getRotation();

        // Add rotation towards target based on vision info
        final double direction = guess.get().direction;
        rotation -= direction * SmartDashboard.getNumber("VisRotGain", 0.0);
        
        rotation = MathUtil.clamp(rotation, -1.0, 1.0);

        drivetrain.drive(OperatorInterface.getSpeed(), rotation);
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.drive(0, 0);
    }
}
