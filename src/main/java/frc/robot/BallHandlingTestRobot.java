// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.cargo.BallHandling;
import frc.robot.cargo.ShootCommand;

/** Simple robot that tests ball handling
 * 
 *  In teleop:
 *  * Nothing should happen
 *  * Check dashboard for "Ball in Conveyor", "Ball in Feeder"
 *    and "Ball Ejected" sensors.
 *    Do they indicate a "Ball" when something is placed at each
 *    location?
 *  * Push A to toggle loading 'on':
 *    Intake should move out/open.
 *    Intake and conveyor should turn on.
 *    Verify that intake (two motors) and conveyor move "in".
 *    Do intake and conveyor motors get up to speed or do
 *    we need to extend the state machine to separate opening/closing
 *    intake from running the motors (like 2016 robot)?
 *  * Push A to toggle loading 'off':
 *    Intake should move in
 *    Intake and conveyor should turn off
 *  * Push A to toggle loading back 'on':
 *    Intake should move out/open
 *    Intake and conveyor should turn on
 *  * Feed one ball, should get pulled in, intake and conveyor keep running
 *  * Feed second ball, intake and conveyor should stop
 *  * Push A to toggle loading 'on/off',
 *    intake should open/close but intake motor and conveyor stay off
 *    when there are already two balls
 * 
 *    TODO Later add check of ball color and reject wrong colored ball?
 * 
 *  * On Dashboard, check if "High" allows moving the shooter angle solenoid
 *    to the high (true) and low (false) setup.
 * 
 *  * Push 'X' to toggle spinner on/off
 *    Spinner should turn on/off
 * 
 *  * Push left bumper to shoot
 *    Spinner should turn on in case it wasn't already running.
 *    Wait until spinner fast enough (95% of desired speed. Is 95% a good number?)
 *    Feeder moves ball up to spinner.
 *    Feeder stops one ball has been ejected.
 *    Spinner either remains on for just 2 seconds
 *    or until 'spinner always on' toggled to off via 'X'
 *
 *  * Try shooting without balls loaded:
 *    Spinner should run up,
 *    feeder runs up,
 *    shot times out after 10 seconds.
 */
public class BallHandlingTestRobot extends TimedRobot
{
    private final BallHandling ball_handling = new BallHandling();
    public final Pneumatics pneumatics = new Pneumatics();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Ball Handling Test *****");

        // TODO Enable, then use Data Log Tool to download file and plot
        // DataLogManager.start();
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("SpinnerState", ball_handling.getShooterState().ordinal());
        // Send out on each period to get better plots
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void disabledInit()
    {
        ball_handling.enable(false);
    }

    @Override
    public void teleopInit()
    {
        OperatorInterface.reset();
        ball_handling.enable(true);
    }

    @Override
    public void teleopPeriodic()
    {
        ball_handling.reverse(OperatorInterface.reverseIntake());

        if (OperatorInterface.toggleLoading())
            ball_handling.toggleLoading();

        if (OperatorInterface.toggleSpinner())
            ball_handling.toggleSpinner();
    
        if (OperatorInterface.doShoot())
            ball_handling.shoot();    
    }

    @Override
    public void autonomousInit()
    {
        new SequentialCommandGroup(new ShootCommand(ball_handling),
                                   new ShootCommand(ball_handling)).schedule();
    }
}
