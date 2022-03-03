// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;

/** Manual Climber control */
public class ManualClimbCommand extends CommandBase
{
    private final Climber climber;

    public ManualClimbCommand(final Climber climber)
    {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute()
    {
        if (OperatorInterface.homing())
            climber.homing();
        else
        {
            if (OperatorInterface.joystick.getLeftBumper())
            {
                // Use Joystick to set 0 .. MAX_EXTENSION
                double desired_extension = ActiveArm.MAX_EXTENSION * (0.5-OperatorInterface.joystick.getRightY() * 0.5);
                climber.setExtension(desired_extension);
            }
            else
               climber.setExtenderVoltage(0.0);    
        }
    
        if (OperatorInterface.toggleArmAngle())
            climber.setAngle(! climber.getAngle());
    }
}
