// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.camera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;
import frc.robot.drivetrain.Drivetrain;

/** Command for rotating to target */
public class RotateToTargetCommand extends CommandBase
{
    private final Drivetrain drivetrain;
    private final PIDController pid = new PIDController(0, 0, 0);
    private final NetworkTableEntry valid, horiz;

    public RotateToTargetCommand(final Drivetrain drivetrain)
    {
        this.drivetrain = drivetrain;
        valid = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tv");
        horiz = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tx");
    }

    public void configure(final double kp, final double ki, final double kd)
    {
        pid.setPID(kp, ki, kd);
    }

    @Override
    public void execute()
    {
        // Get direction to target, the 'measurement', from camera
        final double measurement;
        if (valid.getDouble(0) < 1)
            measurement = 0.0;
        else
            measurement = horiz.getDouble(0);

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
