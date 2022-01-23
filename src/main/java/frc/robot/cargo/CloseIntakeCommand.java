// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Command to close intake */
public class CloseIntakeCommand extends InstantCommand
{
    @Override
    public void initialize()
    {
        System.out.println("Should close intake...");
        // TODO ball_handling.load(false);
    }
} 
