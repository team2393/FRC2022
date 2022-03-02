// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Robot for testing the new REV Robotics Pneumatic Hub
 * 
 *  See teleopPeriodic for options
 */
public class PneumaticHubTestRobot extends TimedRobot
{
    // private PowerDistribution power = new PowerDistribution();

    private final int CHANNELS = 16;

    private int active_channel = 0;

    private Timer timer = new Timer();

    public final Pneumatics pneumatics = new Pneumatics();
    private List<Solenoid> solenoids = new ArrayList<>();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Pneumatic Hub Test *****");

       //  System.out.println("Power distribution board channels: " + power.getNumChannels());

        // Create one solenoid for each channel
        for (int ch=0; ch<CHANNELS; ++ch)
            solenoids.add(new Solenoid(PneumaticsModuleType.REVPH, ch));
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic()
    {
        // **** Option A: "USER" button turns all solenoids on
        // final boolean turn_on = RobotController.getUserButton();
        // for (int ch=0; ch<CHANNELS; ++ch)
        //     solenoids.get(ch).set(turn_on);        

        // *** Option B:
        // Operate intake, shifter, passive arm via OperatorInterface
        if (OperatorInterface.toggleLoading())
            solenoids.get(RobotMap.INTAKE_ARM).toggle();
        
        if (OperatorInterface.shiftHigh())
            solenoids.get(RobotMap.GEAR_SHIFTER).set(true);
        if (OperatorInterface.shiftLow())
            solenoids.get(RobotMap.GEAR_SHIFTER).set(false);

        if (OperatorInterface.toggleArmAngle())
            solenoids.get(RobotMap.ARM_ROTATOR).toggle();

        if (OperatorInterface.doShoot())
            solenoids.get(RobotMap.SHOOTER_ANGLE).toggle();
    }

    @Override
    public void autonomousInit()
    {
        active_channel = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void autonomousPeriodic()
    {
        // Toggle another of the channels 
        if (timer.advanceIfElapsed(0.1))
        {
            solenoids.get(active_channel).toggle();
            active_channel = (active_channel + 1) % CHANNELS;
        }

        // if (CHANNELS == 16)
        // {
        //     // Every 10 seconds, toggle the switchable channel on the Rev power hub
        //     power.setSwitchableChannel((System.currentTimeMillis() / 10000) % 2 == 1);
        // }
    }
}
