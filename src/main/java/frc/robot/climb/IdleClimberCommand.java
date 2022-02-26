// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Keep Climber where it is */
public class IdleClimberCommand extends CommandBase
{
    private final Climber climber;

    public IdleClimberCommand(final Climber climber)
    {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute()
    {
        climber.setExtenderVoltage(0.0);    
    }
}
