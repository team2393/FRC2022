// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cargo.BallHandling;
import frc.robot.cargo.Spinner;

/** Manual hardware test of ball handling
 * 
 *  Left stick 'right' should move conveyor.
 *  Right stick 'right' should move feeder.
 *  Left bumper runs spinner at "SpinnerSetpoint" RPS.
 */
public class ManualBallHandlingTestRobot extends TimedRobot
{
    private final static XboxController joystick = new XboxController(0);

    private WPI_TalonFX conveyor = new WPI_TalonFX(RobotMap.CONVEYOR);
    private WPI_TalonFX feeder = new WPI_TalonFX(RobotMap.FEEDER);
    private Spinner spinner = new Spinner();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Manual Ball Handling Test *****");

        // Apply common settings
        BallHandling.initializeMotor(conveyor);
        BallHandling.initializeMotor(feeder);        
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        // Spinner itself displays speed in RPS on dashboard.
        // Add RPM and details needed to configure/tune the spinner.
        SmartDashboard.putNumber("Spinner Rev", spinner.getPosition());
        SmartDashboard.putNumber("Spinner RPM", spinner.getSpeed() * 60.0);
        SmartDashboard.putNumber("Spinner Voltage", spinner.getVoltage());
        SmartDashboard.putNumber("Spinner Current", spinner.getCurrent()); 
        SmartDashboard.putNumber("Spinner Current Change", spinner.getCurrentChange()); 
        SmartDashboard.putBoolean("Spinner Ball Ejected", spinner.isBallEjected()); 

        // Send out on each period to get better plots
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void teleopPeriodic()
    {
        double voltage = 12.0*joystick.getLeftX();
        conveyor.setVoltage(voltage);
        SmartDashboard.putNumber("ConveyorVoltage", voltage);

        voltage = 12.0*joystick.getRightX();
        feeder.setVoltage(voltage);
        SmartDashboard.putNumber("FeederVoltage", voltage);

        if (joystick.getLeftBumper())
            spinner.run();
        else
            spinner.stop();
    }
}
