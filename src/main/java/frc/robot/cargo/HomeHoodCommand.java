// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to 'home' Hood */
public class HomeHoodCommand extends CommandBase
{
    private final Hood hood;

    private static final double HOMING_OUTPUT = 0.1;

    private enum State
    {
        // First move 'out' to clear switch in case we're already there
        MOVE_OUT,
        // Move 'in' until we hit the switch
        MOVE_IN,
        // Done
        DONE
    }

    private State state;

    public HomeHoodCommand(final Hood hood)
    {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize()
    {
        state = State.MOVE_OUT;
    }

    @Override
    public void execute()
    {
        if (state == State.MOVE_OUT)
        {   // Move out, away from the limit switch?
            if (hood.atHome())
                hood.setOutput(HOMING_OUTPUT);
            else
                state = State.MOVE_IN;
        }
        if (state == State.MOVE_IN)
        {   // Move out, away from the limit switch?
            if (hood.atHome())
            {
                state = State.DONE;
                hood.setOutput(0);
                hood.reset();
            }
            else
                hood.setOutput(-0.5*HOMING_OUTPUT);
        }
    }

    @Override
    public boolean isFinished()
    {
        return state == State.DONE;
    }
} 
