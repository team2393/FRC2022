// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cargo.HomeHoodCommand;
import frc.robot.cargo.Hood;
import frc.robot.cargo.SetHoodCommand;

/** Hood test robot
 * 
 *  [ ] In teleop, calibrate STEPS_PER_PERC,
 *      check direction (forward == out),
 *      MAX_POS_PERC, MAX_PERC_PER_SEC
 * 
 *  [ ] Check homing
 * 
 *  [ ] In auto, use Phoenix Tuner to configure P, D, I
 */
public class HoodTestRobot extends TimedRobot
{
    private final Hood hood = new Hood();
    private final HomeHoodCommand home = new HomeHoodCommand(hood);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Hood Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        
        // Send out on each period to get better plots
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void teleopInit()
    {
        hood.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        // In teleop, manually control hood through joystick
        // except when 'Y' is pressed to home
        if (OperatorInterface.joystick.getYButton())
            home.schedule();
        if (! home.isScheduled())
            hood.setOutput(0.1*OperatorInterface.getSpeed());
    }

    @Override
    public void autonomousInit()
    {
        new SetHoodCommand(hood).schedule();
    }
}
