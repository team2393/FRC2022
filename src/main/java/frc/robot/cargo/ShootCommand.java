// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to shoot */
public class ShootCommand extends CommandBase
{
    // TODO handle the shooting of one ball
    // Do that in here?
    // Or have a designated "BallHandling" subsystem with states like
    // IDLE, SPINUP, FEED, EJECT, SPINDOWN?

    private final Timer simulation = new Timer();

    @Override
    public void initialize()
    {
        System.out.println("Preparing to shoot...");
        // Start spinner..

        simulation.reset();
        simulation.start();
    }

    @Override
    public void execute()
    {
        // Check if spinner fast enough?
        // Then run feeder until ball has been ejected?
    }

    @Override
    public boolean isFinished()
    {
        // Are we done?
        // Pretend it takes 2 seconds
        if (simulation.hasElapsed(2.0))
        {
            System.out.println("Ejected ball");
            return true;
        }
        return false;
    }
} 
