// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivetrain.Drivetrain;

/** Robot for testing/calibrating drivetrain
 * 
 *  [ ] In teleop, check if 'forward' is forward and 'right' is right
 *  [ ] Check if 'shifting' results in indication on pneumatic module
 *  [ ] Check Drivetrain.STEPS_PER_METER calibration.
 *  [ ] Note "Drivetrain meters per rev: ____"
 *      printed out during startup
 *  [ ] Use sysId to determine FF and PID settings.
 *      Setup:
 *      'Drivetrain' two Falcon pairs,
 *      Built-in encoder, 2048 counts/rev, 1:1 gearing.
 *      In "Logger", enter the meters per rev for Units Per Rotoation.
 *  [ ] Update FF, PID and TRACK_WIDTH from sysId results
 *  [ ] Configure PID to meet "Desired Speed" in autonomous
 *  [ ] Test autonomous paths in RapidReactRobot
*/
public class DrivetrainTestRobot extends TimedRobot
{
    private final Drivetrain drivetrain = new Drivetrain();
    public final Pneumatics pneumatics = new Pneumatics();
    
    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Drivetrain Test *****");

        // Allow setting the speed used during auto-no-mouse
        SmartDashboard.setDefaultNumber("Desired Speed", 0.0);

        SmartDashboard.setDefaultNumber("P", 4.0);
        SmartDashboard.setDefaultNumber("I", 0.4);
        SmartDashboard.setDefaultNumber("D", 0.05);
    }

    @Override
    public void robotPeriodic()
    {
        // Display info that we need to adjust drivetrain feed forward and PID settings
        double avg = (drivetrain.getLeftVoltage() + drivetrain.getRightVoltage()) / 2;
        SmartDashboard.putNumber("Voltage",  avg);

        // Make commands and subsystems execute
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit()
    {
        drivetrain.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        // Manually drive robot
        drivetrain.drive(OperatorInterface.getSpeed(), OperatorInterface.getRotation());

        if (OperatorInterface.shiftHigh())
            drivetrain.shiftgear(true);
        if (OperatorInterface.shiftLow())
            drivetrain.shiftgear(false);
    }

    @Override
    public void autonomousPeriodic()
    {
        // Allow adjusting PID from dashboard
        drivetrain.configurePID(SmartDashboard.getNumber("P", 0.0),
                                SmartDashboard.getNumber("I", 0.0),
                                SmartDashboard.getNumber("D", 0.0));
        // Run 'straight forward' at speed set in dashboard
        final double speed = SmartDashboard.getNumber("Desired Speed", 0.0);
        drivetrain.setSpeeds(speed, speed);
    }
}
