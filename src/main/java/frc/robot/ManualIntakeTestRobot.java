package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.cargo.BallHandling;

public class ManualIntakeTestRobot extends TimedRobot
{
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
    public void teleopPeriodic()
    {
        if (OperatorInterface.toggleLoading())
            intake_arm.toggle();
        double voltage = OperatorInterface.getSpeed();
        intake.setVoltage(voltage);
    }
}
