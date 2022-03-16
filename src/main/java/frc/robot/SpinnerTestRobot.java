// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cargo.Spinner;

/** Simple robot that tests the spinner
 * 
 *  [x] In teleop, check "Spinner Rev".
 *      Need to enable teleop to zero the encoder.
 *      Caution, don't touch the joystick! 
 *      Manually turn spinner by 10 revs,
 *      check if "Spinner Rev" shows 10.
 *      If not, adjust Spinner.STEPS_PER_REV.
 *  [x] Control spinner voltage -12..12 through joystick 'speed'
 *      "Forward" must move spinner in "ejection" direction.
 *  [x] Use sysId tool with "General Mechanism",
 *      Two motors, second one inverted,
 *      Built-in encoder, 2048 counts per rev,
 *      log as "Simple" mechanism, Rotations, 1 unit per rotation,
 *      run the tests, get feed forward and PID settings.
 * 
 *  [x] In autonomous, enter desired "SpinnerSetpoint"
 *      and check if spinner gets there
 * 
 *  [ ] Test using the setSpeed() "initial_rampup" code
 * 
 *  [ ] In autonomous, at setpoint, feed a ball
 *      and observe "Spinner Current" and "Spinner Current Change".
 *      See Spinner.getCurrentChange() to detect ejected ball.
 *      Try instead to look for drop in RPM.
 *      Which one is best to detect an ejected ball?
 * 
 *  [ ] Set appropriate threshold and 'keep on filter' time
 *      to get reliable "Ball Ejected" indication
 */
public class SpinnerTestRobot extends TimedRobot
{
    private final Spinner spinner = new Spinner();

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Spinner Test *****");
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        
        SmartDashboard.putBoolean("Spinner Ball Ejected", spinner.isBallEjected()); 
        // Send out on each period to get better plots
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void teleopInit()
    {
        spinner.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        // In teleop, manually control spinner through joystick
        spinner.setOutput(OperatorInterface.getSpeed());
    }

    @Override
    public void autonomousPeriodic()
    {
        // In auto, ask spinner to run at desired speed
        // spinner.setVoltage(SmartDashboard.getNumber("SpinnerSetpoint", 0.0));
        spinner.run();
    }
}
