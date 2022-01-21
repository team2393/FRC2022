// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Command to aim low */
public class AimLowCommand extends InstantCommand
{
    private final Solenoid solenoid;

    public AimLowCommand(final Solenoid solenoid)
    {
        this.solenoid = solenoid;
    }

    @Override
    public void initialize()
    {
        // TODO Check solenoid operation. Also need to update spinner speed?
        System.out.println("Should aim low...");
        solenoid.set(false);
    }
} 
