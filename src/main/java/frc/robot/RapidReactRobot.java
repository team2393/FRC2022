// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.ApplySettingsCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.camera.CameraHelper;
import frc.robot.camera.GuessingUDPClient;
import frc.robot.cargo.BallHandling;
import frc.robot.climb.ArmInOutCommand;
import frc.robot.climb.ClimbSequence;
import frc.robot.climb.Climber;
import frc.robot.climb.ManualClimbCommand;
import frc.robot.climb.SetClimberExtensionCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.StayPutCommand;
import frc.robot.drivetrain.AutoShiftCommand;
import frc.robot.drivetrain.DriveAndRotateToVisionCommand;
import frc.robot.drivetrain.DriveByJoystickCommand;

/** Team 2393 'Rapid React' robot */
public class RapidReactRobot extends TimedRobot
{
    // Robot components
    BallHandling ball_handling = new BallHandling();

    /** Drive motors */
    private final Drivetrain drivetrain = new Drivetrain(ball_handling.pigeon_carrier);

    public final Pneumatics pneumatics = new Pneumatics();

    private final PowerDistribution power = new PowerDistribution();

    public final Climber climber = new Climber();
    
    private final CommandBase manual_climb = new ManualClimbCommand(climber);
    private final CommandBase climb_low = new SetClimberExtensionCommand(climber, "Arm Low", -0.04);    
    private final CommandBase climb_mid = new SetClimberExtensionCommand(climber, "Arm Mid", 0.4);    
    private final CommandBase climb_high = new SetClimberExtensionCommand(climber, "Arm High", 0.85);    
    private final CommandBase arm_out = new ArmInOutCommand(climber, true);
    private final CommandBase arm_in = new ArmInOutCommand(climber, false);
    // private final CommandBase arm_out = new InstantCommand(() -> climber.setAngle(true));
    // private final CommandBase arm_in = new InstantCommand(() -> climber.setAngle(false));
    final private CommandBase climb_sequence = new ClimbSequence(climber, ball_handling);

    /** Camera info client */
    private GuessingUDPClient camera = new GuessingUDPClient();

    /** Commands */
    private final CommandBase joydrive = new DriveByJoystickCommand(drivetrain),
                              auto_shift = new AutoShiftCommand(drivetrain),
                              camera_drive = new DriveAndRotateToVisionCommand(drivetrain, () -> camera.get().direction);

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

        SmartDashboard.putData(auto_shift);

        // Register commands on dashboard
        reset_command.setName("Reset");
        SmartDashboard.putData(reset_command);

        SmartDashboard.putData(new ApplySettingsCommand("Aim High at Line", "high_at_line.dat"));
        SmartDashboard.putData(new ApplySettingsCommand("Aim High Near", "high_near.dat"));
        SmartDashboard.putData(new ApplySettingsCommand("Aim Low Mid", "low_mid.dat"));
        SmartDashboard.putData(new ApplySettingsCommand("Aim Low Near", "low_near.dat"));

        SmartDashboard.putData(climb_sequence);

        // Camera support
        SmartDashboard.putData(camera_drive);
        CameraHelper.registerCommands();

        reset();
    }

    /** Reset drivetrain.. */
    private void reset()
    {
        drivetrain.reset();
        auto_shift.cancel();
        climber.reset();
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
        ball_handling.enable(false);
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

        reset();
        ball_handling.enable(true);

        // Start driving via joystick
        joydrive.schedule();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic()
    {
        // ****** Driving *****************
        // GUI can select camera_drive.
        // If that's not running, re-schedule the plain joydrive
        if (! camera_drive.isScheduled())
            joydrive.schedule();

        // auto_shift can be enabled on dashboard, and always allow manual shifting
        // TODO Replace manual shift with scheduling/cancelling auto_shift
        if (OperatorInterface.shiftHigh())
            // auto_shift.schedule();
            drivetrain.shiftgear(true);
        if (OperatorInterface.shiftLow())
            // auto_shift.cancel();
            drivetrain.shiftgear(false);

        // ****** Shooting (but peaceful) *****************
        ball_handling.reverse(OperatorInterface.reverseIntake());

        if (OperatorInterface.toggleLoading())
            ball_handling.toggleLoading();

        if (OperatorInterface.toggleSpinner())
            ball_handling.toggleSpinner();
    
        if (OperatorInterface.doShoot())
            ball_handling.shoot();
            
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
    }

    /** This function is called when entering auto-no-mouse mode */
    @Override
    public void autonomousInit()
    {
        System.out.println("Auto-No-Mouse");

        reset();
        ball_handling.enable(true);
        
        // Start selected auto command
        auto_options.getSelected().schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        // Typically empty since it's all done
        // within the command started in autonomousInit()...

        climber.homing();
    }
}
