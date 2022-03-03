// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to shoot */
public class ShootCommand extends CommandBase
{
    final private BallHandling ball_handling;

    public ShootCommand(final BallHandling ball_handling)
    {
        this.ball_handling = ball_handling;
    }

    @Override
    public void initialize()
    {
        ball_handling.shoot();
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return ball_handling.hasShot();
    }
} 
