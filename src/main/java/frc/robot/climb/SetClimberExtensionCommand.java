// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Set Climber to fixed extension */
public class SetClimberExtensionCommand extends CommandBase
{
    // Could shorten this as RunCommand(() -> climber.setExtension(extension), climber);

    private final Climber climber;
    private final double extension;

    public SetClimberExtensionCommand(final Climber climber, final double extension)
    {
        this.climber = climber;
        this.extension = extension;
        addRequirements(climber);
    }

    @Override
    public void execute()
    {
        climber.setExtension(extension);    
    }
}
