// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Home Climber */
public class HomeCommand extends CommandBase
{
    private final Climber climber;
    private boolean done = false;

    public HomeCommand(final Climber climber)
    {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute()
    {
        done = climber.homing();
    }

    @Override
    public boolean isFinished()
    {
        return done;
    }
}
