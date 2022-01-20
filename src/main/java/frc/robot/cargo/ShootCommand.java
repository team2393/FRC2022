// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to shoot */
public class ShootCommand extends CommandBase
{
    // TODO handle the shooting of one ball
    // Do that in here?
    // Or have a designated "BallHandling" subsystem with states like
    // IDLE, SPINUP, FEED, EJECT, SPINDOWN?

    @Override
    public void initialize()
    {
        System.out.println("Should prepare to shoot...");
        // Start spinner..
    }

    @Override
    public void execute()
    {
        // Check if spinner fast enough?
        // Then run feeder until ball has been ejected?
        System.out.println("Ejecting ball");
    }

    @Override
    public boolean isFinished()
    {
        // Are we done??
        return true;
    }
} 
