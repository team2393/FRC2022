// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.cargo.BallHandling;

/** Test intake solenoid and motors
 * 
 *  In teleop,
 *  [ ] connect only one motor at a time and test direction
 *  [ ] check direction of solenoid.
 *  [ ] Determine good BallHandling.INTAKE_VOLTAGE,
 *  [ ] then try autonomous
 */
public class ManualIntakeTestRobot extends TimedRobot
{
    public final Pneumatics pneumatics = new Pneumatics();
    private WPI_TalonSRX intake = new WPI_TalonSRX(RobotMap.LEFT_INTAKE);
    private WPI_TalonSRX secondary_intake = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE);
    private Solenoid intake_arm = new Solenoid(RobotMap.PCM_TYPE, RobotMap.INTAKE_ARM);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Manual Intake Test *****");

        // Intake spinners can coast
        BallHandling.initializeIntakeMotor(intake);
        BallHandling.initializeIntakeMotor(secondary_intake);
        secondary_intake.setInverted(true);
        secondary_intake.follow(intake);
    }

    @Override
    public void disabledInit()
    {
        intake_arm.set(false);
    }

    @Override
    public void teleopPeriodic()
    {
        if (OperatorInterface.toggleLoading())
            intake_arm.toggle();

        double voltage = OperatorInterface.getSpeed();
        intake.setVoltage(voltage);
        SmartDashboard.putNumber("Intake Voltage", voltage);
    }

    @Override
    public void autonomousPeriodic()
    {
        intake_arm.set(true);
        intake.setVoltage(BallHandling.INTAKE_VOLTAGE);
    }
}
