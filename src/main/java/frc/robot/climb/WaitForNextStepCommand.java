// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;

/** Command to wait for button presses that move us to next step in a sequence */
public class WaitForNextStepCommand extends CommandBase 
{
    @Override
    public boolean isFinished()
    {
        // TODO Come up with better button
        return OperatorInterface.joystick.getYButtonPressed();
        // && OperatorInterface.buttons.getRawButton(1);
    }
}
 