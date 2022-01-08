// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** Team 2393 'Rapid React' robot */
public class RapidReactRobot extends TimedRobot
{
    // TODO DriveTrain, ...
    private final Servo test_servo = new Servo(0);

    /** Options shown on dashboard for selecting what to do in auto-no-mouse mode  */
    private final SendableChooser<CommandBase> auto_options = new SendableChooser<>();

    /** This function runs once on robot startup. */
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Rapid React *****");

        // TODO Configure motors etc.

        // TODO Load autonomous options, eventually likely commands to follow trajectory etc.
        auto_options.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
        auto_options.addOption("Hello", new PrintCommand("Hello!"));
        SmartDashboard.putData("Auto Options", auto_options);
    }

    /** This function is called all the time regardless of mode. */
    @Override
    public void robotPeriodic()
    {
        // Run one 'step' of the command scheduler.
        // This is what allows us to use commands.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit()
    {
        System.out.println("Disabled");
    }

    /** This function is while the robot is disabled. */
    @Override
    public void disabledPeriodic()
    {
        // Doing nothing
    }

    /** This function is called when starting teleop mode. */
    @Override
    public void teleopInit()
    {
        System.out.println("Teleop");
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic()
    {
        // TODO Read UI, ...
        test_servo.setAngle(90.0 + OperatorInterface.getTurn() * 75.0);
    }

    /** This function is called when entering auto-no-mouse mode */
    @Override
    public void autonomousInit()
    {
        auto_options.getSelected().schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        // Typically nothing do here to since it's all done
        // within the command started in autonomousInit()...
    }
}
