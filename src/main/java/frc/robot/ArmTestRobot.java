// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.climb.ActiveArm;
import frc.robot.climb.PassiveArm;

/** Robot for testing/calibrating arm
 * 
 *  Start out with active arm all pulled "in" (no extension) and passive arm "up".
 *  Encoders are reset to zero when starting teleop.
 * 
 *  Recipe for passive arm
 *  1) In teleop, right bumper should toggle passive arm up/down
 *  5) In auto mode, arm should go up
 * 
 *  Recipe for extension of active arm
 *  1) In teleop, right stick forward/backwards
 *     should extend arm out/in.
 *  2) Does right joystick forward extend "out"? Otherwise invert extender motor
 *  3) Check limit switch.
 *     Does it indicate "retracted" when at limit?
 *     Extender motor should not move further "in" when fully retracted.
 *  4) Calibrate extender sensor EXTENDER_COUNTS_PER_METER:
 *     Set EXTENDER_COUNTS_PER_METER = 1,
 *     extend arm some way, measure extension and compute
 *     counts / that_extension
 *  5) Check MAX_EXTENSION
 *     Set MAX_EXTENSION = 0
 *     In teleop, determine maximum extension,
 *     then set MAX_EXTENSION to that value (minus a few centimeters?).
 *     Re-deploy, and check if teleop is limited to that extension.
 *  6) Set EXTENDER_VOLTAGE
 *     Moving extender in teleop, determine a good voltage to use for homing
 *  7) Test homing
 *     In teleop, extend arm, then either hold 'X' or
 *     set "Desired Extension" on dashboard to zero,
 *     then switch to autonomous.
 *     Either way, verify that arm moves all "in",
 *     and reports exactly 0 m extension when done
 *  8) Back in teleop,
 *     with robot hanging on arm, lifing up and letting it back down,
 *     determine the (negative) voltage needed to just hold the robot against gravity,
 *     configure FF_HOLDING_VOLTAGE.
 *  9) In autonomous, enter a "Desired Extension" on dashboard
 *     and adjust P, I, D, Max Arm V to get there.
 * 10) Try various "Desired Extension" on dashboard.
 * 11) Fix P, I, D, Max V settings in ActiveArm
 * 12) Replace PIDController in ActiveArm with ProfiledPIDController,
 *     adjust speed of ProfiledPIDController.
 * 
 * 13) Change 'arm' from RobotMap.LEFT_ARM_... to RobotMap.RIGHT_ARM_...,
 *     check if that works with the same settings
 * 14) Check if both arms can be moved by each running its own ActiveArm instance.
 *     Starting from a homed state, do both ProfiledPIDControllers come up
 *     with the same setpoint profiles and track each other?
 *     Otherwise create a new ActiveArmFollower that uses a plain PID
 *     where the setpoint is updated from the primary ActiveArm.
 */
public class ArmTestRobot extends TimedRobot
{
    private final ActiveArm arm = new ActiveArm(8, RobotMap.LEFT_ARM_RETRACTED);
    // private final PassiveArm passive = new PassiveArm();
    
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Arm Test *****");

        SmartDashboard.setDefaultNumber("Desired Extension", 0.0);

        SmartDashboard.setDefaultNumber("P", 100.0);
        SmartDashboard.setDefaultNumber("I", 5.0);
        SmartDashboard.setDefaultNumber("D", 0.0);
        SmartDashboard.setDefaultNumber("Max Arm V", 8.0);
    }

    @Override
    public void robotPeriodic()
    {
        // Make commands and subsystems execute
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Arm Extension", arm.getExtension());
        SmartDashboard.putBoolean("Arm Retracted", arm.isRetracted());
    }

    @Override
    public void teleopInit()
    {
        arm.reset();
        // passive.reset();
        OperatorInterface.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        // Manually operate arm
        // X to home, else move with right stick
        if (OperatorInterface.joystick.getXButton())
        {
            if (arm.homing())
                SmartDashboard.putNumber("Extender Voltage", 0);         
            else
                SmartDashboard.putNumber("Extender Voltage", -ActiveArm.EXTENDER_VOLTAGE); 
        }
        else if (OperatorInterface.joystick.getLeftBumper())
        {
            // Use Joystick to set 0 .. MAX_EXTENSION
            double desired_extension = ActiveArm.MAX_EXTENSION * (0.5-OperatorInterface.joystick.getRightY() * 0.5);
            SmartDashboard.putNumber("Desired Extension", desired_extension);
            arm.setExtension(desired_extension);
        }
        else
        {
            // Use right stick 'forward' for 'up', extend arm 'out'
            final double joystick = MathUtil.applyDeadband(-OperatorInterface.joystick.getRightY(), 0.05);
            final double voltage = joystick * 12.0;
            SmartDashboard.putNumber("Extender Voltage", voltage); 
            arm.setExtenderVoltage(voltage);
        }

        // Use right bumper to control up/down angle
        // if (OperatorInterface.toggleArmAngle())
        //     passive.setAngle(! passive.getAngle());
    }

    @Override
    public void autonomousInit()
    {
        arm.reset();
    }
    
    @Override
    public void autonomousPeriodic()
    {
        // passive.setAngle(false);

        // Manually enter desired extension, and allow tuning PID
        final double desired_extension = SmartDashboard.getNumber("Desired Extension", 0.0);    
        arm.configurePID(SmartDashboard.getNumber("P", 0.0),
                         SmartDashboard.getNumber("I", 0.0),
                         SmartDashboard.getNumber("D", 0.0),
                         SmartDashboard.getNumber("Max Arm V", 6.0));
        arm.setExtension(desired_extension);
    }
}
