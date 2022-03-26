// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that rotates to a heading */
public class RotateToHeadingCommand extends CommandBase
{
    private final Drivetrain drivetrain;
    private double desired_degrees;
    private final PIDController pid = new PIDController(0, 0, 0);
    private double max = 0.5;
    private final Timer timer = new Timer();

    /** @param drivetrain Drivetrain to control
     *  @param heading Target heading [degrees]
     */
    public RotateToHeadingCommand(final Drivetrain drivetrain, final double desired_degrees)
    {
        this.drivetrain = drivetrain;
        this.desired_degrees = desired_degrees;
        addRequirements(drivetrain);
        pid.setTolerance(1.0);
    }

    @Override
    public void initialize()
    {
        timer.stop();
        timer.reset();
        timer.start();
    }

    // For RotateToHeadingTestRobot
    public void configure(final double heading,
                          final double tolerance,
                          final double kp, final double ki, final double kd,
                          final double max)
    {
        desired_degrees = heading;
        pid.setTolerance(tolerance);
        pid.setPID(kp, ki, kd);
        this.max = max; 
    }

    @Override
    public void execute()
    {
        double rotation = pid.calculate(drivetrain.getHeading(), desired_degrees);
        rotation = MathUtil.clamp(rotation, -max, max);
        drivetrain.driveDirect(0, rotation);
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.drive(0, 0);
    }

    @Override
    public boolean isFinished()
    {
        return pid.atSetpoint() || timer.hasElapsed(50.0);
    }
}
