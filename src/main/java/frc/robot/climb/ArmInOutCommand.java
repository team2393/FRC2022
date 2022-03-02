// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to move passive arm in/out */
public class ArmInOutCommand extends CommandBase
{
    private final Climber climber;
    private final boolean in_or_out;

    public ArmInOutCommand(final Climber climber, final boolean in_or_out)
    {
        this.climber = climber;
        this.in_or_out = in_or_out;
    }

    @Override
    public void initialize() 
    {
        climber.setAngle(in_or_out);
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}       
    
