// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DriveByJoystickCommand;

/** Team 2393 'Rapid React' robot */
public class RapidReactRobot extends TimedRobot
{
    // Robot components

    /** Drive motors */
    private final Drivetrain drivetrain = new Drivetrain();

    /** Commands */
    private final CommandBase joydrive = new DriveByJoystickCommand(drivetrain);
    
    /** Options shown on dashboard for selecting what to do in auto-no-mouse mode  */
    private final SendableChooser<CommandBase> auto_options = new SendableChooser<>();

    /** This function runs once on robot startup. */
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Rapid React *****");

        // Populate and publish autonomous options
        AutoOptions.populate(auto_options, drivetrain);
        SmartDashboard.putData("Auto Options", auto_options);

        // Handle the drivetrain via commands.
        // By default, have it stop.
        // Otherwise, DriveByJoystickCommand or an autonomous move command control it.
        final CommandBase stay_put = new RunCommand(() -> drivetrain.drive(0, 0), drivetrain);
        drivetrain.setDefaultCommand(stay_put);

        // Register commands on dashboard: Reset stuff
        final CommandBase reset_command = new InstantCommand(this::reset);
        reset_command.setName("Reset");
        SmartDashboard.putData(reset_command);
    }

    /** Reset drivetrain.. */
    private void reset()
    {
        drivetrain.reset();

        // TODO Is there more to reset?
    }

    /** This function is called all the time regardless of mode. */
    @Override
    public void robotPeriodic()
    {
        // Run one 'step' of the command scheduler.
        // This is what allows us to use commands.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once when the robot becomes disabled. */
    @Override
    public void disabledInit()
    {
        System.out.println("Disabled");
    }

    /** This function is called while the robot is disabled. */
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

        // Start driving via joystick
        joydrive.schedule();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic()
    {
    }

    /** This function is called when entering auto-no-mouse mode */
    @Override
    public void autonomousInit()
    {
        System.out.println("Auto-No-Mouse");
        // Start selected auto command
        auto_options.getSelected().schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        // Typically empty since it's all done
        // within the command started in autonomousInit()...
    }
}
