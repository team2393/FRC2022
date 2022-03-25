// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.ApplySettingsCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.camera.AutoFireCommand;
import frc.robot.cargo.BallHandling;
import frc.robot.cargo.HomeHoodCommand;
import frc.robot.cargo.Hood;
import frc.robot.cargo.SetHoodCommand;
import frc.robot.cargo.BallHandling.ShooterStates;
import frc.robot.climb.ArmInOutCommand;
import frc.robot.climb.ClimbSequence;
import frc.robot.climb.Climber;
import frc.robot.climb.HomeCommand;
import frc.robot.climb.IdleClimberCommand;
import frc.robot.climb.ManualClimbCommand;
import frc.robot.climb.SetClimberExtensionCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.StayPutCommand;
import frc.robot.led.BurstCommand;
import frc.robot.led.LEDStrip;
import frc.robot.led.RainbowCommand;
import frc.robot.led.SetColorCommand;
import frc.robot.led.TwoColorSwapCommand;
import frc.robot.led.TwoColorThirdsSwapCommand;
import frc.robot.drivetrain.AutoShiftCommand;
import frc.robot.drivetrain.DriveByJoystickCommand;

/** Team 2393 'Rapid React' robot */
public class RapidReactRobot extends TimedRobot
{
    private final LEDStrip strip = new LEDStrip();
    private final CommandBase led_idle = new TwoColorThirdsSwapCommand(strip,  3.0, Color.kGold, Color.kGreen);
    private final CommandBase led_auto = new RainbowCommand(strip);
    private final CommandBase led_loaded0 = new SetColorCommand(strip, 0, 10, 0);
    private final CommandBase led_loaded1 = new TwoColorThirdsSwapCommand(strip,  0.5, Color.kGold, Color.kGreen);
    private final CommandBase led_loaded2 =  new TwoColorSwapCommand(strip, 0.1, Color.kRed, Color.kBlue);
    private final CommandBase led_fire = new BurstCommand(strip,  0.05, Color.kBlueViolet);

    // Robot components
    private final PowerDistribution power = new PowerDistribution();
    public final Pneumatics pneumatics = new Pneumatics();
    private final BallHandling ball_handling = new BallHandling(power::setSwitchableChannel);

    private final Hood hood = new Hood();

    /** Drive motors */
    private final Drivetrain drivetrain = new Drivetrain(ball_handling.pigeon_carrier);


    public final Climber climber = new Climber();
    
    private final CommandBase climb_idle = new IdleClimberCommand(climber);
    private final CommandBase manual_climb = new ManualClimbCommand(climber);
    private final CommandBase climb_low = new SetClimberExtensionCommand(climber, "Arm Low", -0.04);    
    private final CommandBase climb_mid = new SetClimberExtensionCommand(climber, "Arm Mid", 0.4);    
    private final CommandBase climb_high = new SetClimberExtensionCommand(climber, "Arm High", 0.85);    
    private final CommandBase arm_out = new ArmInOutCommand(climber, true);
    private final CommandBase arm_in = new ArmInOutCommand(climber, false);
    // private final CommandBase arm_out = new InstantCommand(() -> climber.setAngle(true));
    // private final CommandBase arm_in = new InstantCommand(() -> climber.setAngle(false));
    private final CommandBase climb_sequence = new ClimbSequence(climber, ball_handling);

    /** Camera info client */
    private final CommandBase auto_fire = new AutoFireCommand(drivetrain, ball_handling);

    /** Commands */
    private final CommandBase joydrive = new DriveByJoystickCommand(drivetrain),
                              auto_shift = new AutoShiftCommand(drivetrain);

    private final CommandBase reset_command = new InstantCommand()
    {
        public void initialize()
        {
            reset();
        };
        
        public boolean runsWhenDisabled()
        {
            return true;
        };
    };
    
    /** Options shown on dashboard for selecting what to do in auto-no-mouse mode  */
    private final SendableChooser<CommandBase> auto_options = new SendableChooser<>();
    
    // private final NetworkTableEntry nt_high = SmartDashboard.getEntry("High");

    
    /** This function runs once on robot startup. */
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Rapid React *****");

        // Reset power distribution hub
        power.clearStickyFaults();
        power.resetTotalEnergy();

        // Populate and publish autonomous options
        AutoOptions.populate(auto_options, drivetrain, ball_handling);
        SmartDashboard.putData("Auto Options", auto_options);

        // Handle the drivetrain via commands.
        // By default, have it stop.
        // Otherwise, DriveByJoystickCommand or an autonomous move command control it.
        drivetrain.setDefaultCommand(new StayPutCommand(drivetrain));

        // By default, leave climber where it is
        climber.setDefaultCommand(climb_idle);
        
        // Register commands on dashboard
        reset_command.setName("Reset");
        SmartDashboard.putData(auto_shift);
        SmartDashboard.putData(reset_command);

        SmartDashboard.putData(new ApplySettingsCommand("Aim High at Line", "high_at_line.dat"));
        SmartDashboard.putData(new ApplySettingsCommand("Aim High Near", "high_near.dat"));
        SmartDashboard.putData(new ApplySettingsCommand("Aim Low Mid", "low_mid.dat"));
        SmartDashboard.putData(new ApplySettingsCommand("Aim Low Near", "low_near.dat"));

        SmartDashboard.putData(climb_sequence);

        reset();

        led_idle.schedule();
    }

    /** Reset drivetrain.. */
    private void reset()
    {
        drivetrain.reset();
        auto_shift.cancel();
        climber.reset();
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
        ball_handling.enable(false);
        drivetrain.brake(false);
        led_idle.schedule();
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
        reset();
        ball_handling.enable(true);
        drivetrain.brake(true);

        // Start driving via joystick
        joydrive.schedule();

        new HomeHoodCommand(hood).andThen(new SetHoodCommand(hood)).schedule();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic()
    {
        // ****** Driving *****************
        // auto_shift can be enabled on dashboard, and always allow manual shifting
        if (OperatorInterface.shiftHigh())
            // auto_shift.schedule();
            drivetrain.shiftgear(true);
        if (OperatorInterface.shiftLow())
            // auto_shift.cancel();
            drivetrain.shiftgear(false);

        // ****** Camera **********************************
        if (OperatorInterface.startFollowingCamera())
            auto_fire.schedule();
        if (OperatorInterface.stopFollowingCamera())
            joydrive.schedule();

        // ****** Shooting (but peaceful) *****************
        ball_handling.reverse(OperatorInterface.reverseIntake());

        if (OperatorInterface.toggleLoading())
            ball_handling.toggleLoading();

        if (OperatorInterface.toggleSpinner())
            ball_handling.toggleSpinner();
    
        if (OperatorInterface.doShoot())
            ball_handling.shoot();

        // if (OperatorInterface.HiLoPressed())
        // {
        //     // Toggle both hi/lo and set spinner
        //     if (nt_high.getBoolean(false))
        //     {
        //         nt_high.setBoolean(false);
        //         SmartDashboard.putNumber("SpinnerSetpoint", 60);
        //     }
        //     else
        //     {
        //         nt_high.setBoolean(true);
        //         SmartDashboard.putNumber("SpinnerSetpoint", 70);
        //     }

        // }
            
        // ****** Climbing *****************
        if (OperatorInterface.armManualPressed())
            manual_climb.schedule();
        
        if (OperatorInterface.armHighPressed())
            climb_high.schedule();
        
        if (OperatorInterface.armMidPressed())
            climb_mid.schedule();

        if (OperatorInterface.armLowPressed())
            climb_low.schedule();

        if (OperatorInterface.armInPressed())
            arm_in.schedule();

        if (OperatorInterface.armOutPressed())
            arm_out.schedule();
        
        if (OperatorInterface.startClimbSequence())
            climb_sequence.schedule();
            
        if (OperatorInterface.stopClimbSequence())
            climb_sequence.cancel();

        
        // ****** LEDs *****************
        if (ball_handling.getShooterState() != ShooterStates.IDLE)
            led_fire.schedule();
        else
        {
            final int balls = ball_handling.getBallCount();
            if (balls == 2)
                led_loaded2.schedule();
            else if (balls == 1)
                led_loaded1.schedule();
            else
                led_loaded0.schedule();
        } 
    }

    /** This function is called when entering auto-no-mouse mode */
    @Override
    public void autonomousInit()
    {
        reset();
        drivetrain.brake(true);
        ball_handling.enable(true);

        // Start selected auto command
        auto_options.getSelected().schedule();
        new HomeCommand(climber).schedule();
        new HomeHoodCommand(hood).andThen(new SetHoodCommand(hood)).schedule();
        led_auto.schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        // Typically empty since it's all done
        // within the command started in autonomousInit()...
    }
}
