// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.led.CycleColorListCommand;
import frc.robot.led.TurnIndicatorCommand;
import frc.robot.led.LEDStrip;
import frc.robot.led.OscillateCommand;
import frc.robot.led.RainbowCommand;
import frc.robot.led.SetColorCommand;
import frc.robot.led.TrafficLightCommand;

/** Robot that tests LED strip */
public class LEDStripTestRobot extends TimedRobot
{
    private final LEDStrip strip = new LEDStrip();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 LED Strip Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        // Make the command system do its thing
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit()
    {
        strip.setAll(5, 5, 10);
    }

    @Override
    public void teleopInit()
    {
        new TurnIndicatorCommand(strip).schedule();
    }

    @Override
    public void autonomousInit()
    {
        // Individual commands...
        CommandBase rainbow = new RainbowCommand(strip);

        CommandBase team_colors = new CycleColorListCommand(strip, 0.2,
                                                            Color.kGold,
                                                            Color.kGreen);

        // .. combined into sequence ..
        SequentialCommandGroup sequence =
            new SequentialCommandGroup(
                new SetColorCommand(strip, 255, 0, 0),
                new WaitCommand(1.0),

                new SetColorCommand(strip, 0, 255, 0),
                new WaitCommand(1.0),

                new SetColorCommand(strip, 0, 0, 255),
                new WaitCommand(1.0),

                // Run group until one of them finishes...
                new ParallelRaceGroup(new OscillateCommand(strip), new WaitCommand(3.0)),

                new ParallelRaceGroup(rainbow, new WaitCommand(5.0)),

                new SetColorCommand(strip, 10, 10, 10),
                new WaitCommand(2),

                team_colors.withTimeout(5.0),

                new TrafficLightCommand(strip, 1.0).withTimeout(6.0),
                
                new SetColorCommand(strip, 5, 0, 0),
                new PrintCommand("Done")
                );

        sequence.schedule();
    }
}
