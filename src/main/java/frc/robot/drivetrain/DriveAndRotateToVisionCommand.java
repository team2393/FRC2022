// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;

/** Command that rotates based on vision info */
public class DriveAndRotateToVisionCommand extends CommandBase
{
    private final Drivetrain drivetrain;
    private final DoubleSupplier direction_supplier;

    /** @param drivetrain Drivetrain to control
     *  @param direction_supplier We'll call that to fetch the current camera info
     */
    public DriveAndRotateToVisionCommand(final Drivetrain drivetrain, final DoubleSupplier direction_supplier)
    {
        this.drivetrain = drivetrain;
        this.direction_supplier = direction_supplier;
        addRequirements(drivetrain);

        SmartDashboard.setDefaultNumber("VisRotGain", 0.0);
    }

    @Override
    public void execute()
    {
        // Start with rotation from joystick
        double rotation = OperatorInterface.getRotation();

        // Add rotation towards target based on vision info
        final double direction = direction_supplier.getAsDouble();
        // Positive direction means target is to the right
        // Positive rotation means rotate clockwise
        final double camera_correction = direction * SmartDashboard.getNumber("VisRotGain", 0.0);
        // Limit the contribution of the camera
        rotation += MathUtil.clamp(camera_correction, -0.5, 0.5);
        
        // Limit overall rotation
        rotation = MathUtil.clamp(rotation, -1.0, 1.0);

        drivetrain.drive(OperatorInterface.getSpeed(), rotation);
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.drive(0, 0);
    }
}
