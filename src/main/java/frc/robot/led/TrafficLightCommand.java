// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.led.LEDStrip.N;

/** Command that simulates traffic light */
public class TrafficLightCommand extends CommandBase
{
    private final LEDStrip strip;
    private final double period;
    private final Timer timer = new Timer();

    private enum State { RED, GREEN, YELLOW };
    private State state;
        
    public TrafficLightCommand(final LEDStrip strip, final double period)
    {
        this.strip = strip;
        this.period = period;
        addRequirements(strip);
    }

    @Override
    public void initialize()
    {
        timer.stop();
        timer.reset();
        timer.start();
        state = State.RED;
    }

    @Override
    public void execute()
    {
        if (timer.advanceIfElapsed(period))
        {
            if (state == State.RED)
                state = State.GREEN;
            else if (state == State.GREEN)
                state = State.YELLOW;
            else // must be YELLOW
                state = State.RED;
        }

        strip.setAll(Color.kBlack);
        if (state == State.RED)
        {
            for (int i=0; i<N/3; ++i)
                strip.set(i, Color.kRed);
        }
        else if (state == State.GREEN)
        {
            for (int i=2*N/3; i<N; ++i)
                strip.set(i, Color.kGreen);
        }
        else
        {
            for (int i=N/3; i<2*N/3; ++i)
                strip.set(i, Color.kYellow);
        }
    }
}
