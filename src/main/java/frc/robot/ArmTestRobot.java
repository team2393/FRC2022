// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.climb.Arm;

/** Robot for testing/calibrating arm
 * 
 *  Start out with arm all pulled "in" (no extension) and "up".
 *  Encoders are reset to zero when starting teleop.
 * 
 *  Recipe for extension
 *  1) In teleop, right stick forward/backwards
 *     should extend arm out/in.
 *  2) Does right joystick forward extend "out"? Otherwise invert extender motor
 *  3) Calibrate extender sensor EXTENDER_COUNTS_PER_METER:
 *     Set EXTENDER_COUNTS_PER_METER = 1,
 *     extend arm some way, measure extension and compute
 *     counts / that_extension
 *  4) Check limit switch.
 *     Does it indicate "retracted" when at limit?
 *     Extender motor should not move further "in" when fully retracted.
 *  5) Check MAX_EXTENSION
 *     Set MAX_EXTENSION = 0
 *     In teleop, determine maximum extension,
 *     then set MAX_EXTENSION to that value (minus a few centimeters?).
 *     Re-deploy, and check if teleop is limited to that extension.
 *  6) Set EXTENDER_VOLTAGE
 *     Moving extender in teleop, determine a good voltage to use for homing
 *  7) Test homing
 *     In teleop, extend arm.
 *     Then switch to autonomous and verify that arm moves all "in",
 *     and reports exactly 0 m extension when done
 *
 *  Recipe for angle
 *  1) In teleop, right bumber should toggle arm up/down
 *  5) In auto mode, arm should go up
 */
public class ArmTestRobot extends TimedRobot
{
    private final Arm arm = new Arm();
    
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Arm Test *****");

        SmartDashboard.setDefaultNumber("Desired Angle", 0.0);

        SmartDashboard.setDefaultNumber("P", 0.0);
        SmartDashboard.setDefaultNumber("I", 0.0);
        SmartDashboard.setDefaultNumber("D", 0.0);
        SmartDashboard.setDefaultNumber("Max Arm V", 6.0);
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
        arm.reset();
        OperatorInterface.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        // Manually operate arm at some low voltage
        
        // Use right forward/back to extend arm out/in
        double voltage = OperatorInterface.extendArm() * 4.0;
        SmartDashboard.putNumber("Extender Voltage", voltage); 
        arm.setExtenderVoltage(voltage);

        // Use left forward/back to control up/down angle
        if (OperatorInterface.toggleArmAngle())
            arm.setAngle(! arm.getAngle());
    }

    @Override
    public void autonomousInit()
    {
    }
    
    @Override
    public void autonomousPeriodic()
    {
        arm.homing();
        arm.setAngle(false);        
    }
}
