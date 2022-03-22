// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;

/** Command that drives based on joystick input */
public class DriveByJoystickCommand extends CommandBase
{
    private final Drivetrain drivetrain;

    // Preserve battery and reduce gear wear:
    // Slew by 3 per second, i.e. 1/3 sec until full speed
    private final SlewRateLimiter speed_limiter = new SlewRateLimiter(1);
    private final SlewRateLimiter turn_limiter = new SlewRateLimiter(3);

    public DriveByJoystickCommand(final Drivetrain drivetrain)
    {
        setName("Joydrive");
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {
        speed_limiter.reset(0);
        turn_limiter.reset(0);
    }

    @Override
    public void execute()
    {
        drivetrain.drive(speed_limiter.calculate(OperatorInterface.getSpeed()),
                         turn_limiter.calculate(OperatorInterface.getRotation()));
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.drive(0, 0);
    }
}
