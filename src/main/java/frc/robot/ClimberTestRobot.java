// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.climb.ActiveArm;
import frc.robot.climb.Climber;
import frc.robot.drivetrain.Drivetrain;

/** Robot for testing arms */
public class ClimberTestRobot extends TimedRobot
{
    private final Climber climber = new Climber();
    private final TalonSRX pigeon_carrier = new TalonSRX(RobotMap.LEFT_INTAKE);
    private final Drivetrain drivetrain = new Drivetrain(pigeon_carrier);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Climber Test *****");

        SmartDashboard.setDefaultNumber("Desired Extension", 0.0);
    }

    @Override
    public void robotPeriodic()
    {
        // Make commands and subsystems execute
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit()
    {
        climber.reset();
        OperatorInterface.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        // Manually operate arm
        // X to home, else move with right stick
        if (OperatorInterface.joystick.getXButton())
        {
            climber.homing();
            drivetrain.drive(0.0, 0.0);
        }
        else if (OperatorInterface.joystick.getLeftBumper())
        {
            // Use Joystick to set 0 .. MAX_EXTENSION
            double desired_extension = ActiveArm.MAX_EXTENSION * (0.5-OperatorInterface.joystick.getRightY() * 0.5);
            SmartDashboard.putNumber("Desired Extension", desired_extension);
            climber.setExtension(desired_extension);
            drivetrain.drive(0.0, 0.0);
        }
        else
        {
            climber.setExtenderVoltage(0.0);
            drivetrain.drive(0.5*OperatorInterface.getSpeed(),
                             0.5*OperatorInterface.getRotation());
        }

        // Use right bumper to control up/down angle
        if (OperatorInterface.toggleArmAngle())
            climber.setAngle(! climber.getAngle());
    }

    @Override
    public void autonomousInit()
    {
    }
    
    @Override
    public void autonomousPeriodic()
    {
        // Manually enter desired extension
        final double desired_extension = SmartDashboard.getNumber("Desired Extension", 0.0);    
        climber.setExtension(desired_extension);
    }
}
