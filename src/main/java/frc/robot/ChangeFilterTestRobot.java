// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.KeepOnFilter;

/** Demo of change filter */
public class ChangeFilterTestRobot extends TimedRobot
{
    /** Get change of a signal
     * 
     *  Takes first derivative, i.e. basically (current - last),
     *  but does that as an average over the last N readings.
     *  Sample period is 1/50  = 0.02 seconds
     */
    private final LinearFilter change_filter10 = LinearFilter.backwardFiniteDifference(1, 10, 0.02);
    private final LinearFilter change_filter5 = LinearFilter.backwardFiniteDifference(1, 5, 0.02);
    private final LinearFilter change_filter3 = LinearFilter.backwardFiniteDifference(1, 3, 0.02);
    private final LinearFilter change_filter2 = LinearFilter.backwardFiniteDifference(1, 2, 0.02);
    private final LinearFilter highpass = LinearFilter.highPass(0.1, 0.02);
 
    private final KeepOnFilter keep_on = new KeepOnFilter(2.0);
    private final KeepOnFilter ball_ejected = new KeepOnFilter(2.0);

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Change Filter Test *****");
    }

    @Override
    public void teleopPeriodic()
    {
        final double current_value = OperatorInterface.getSpeed();

        final double change10 = change_filter10.calculate(current_value) / 10.0;
        final double change5  = change_filter5.calculate(current_value) / 5.0;
        final double change3  = change_filter3.calculate(current_value) / 3.0;
        final double change2  = change_filter2.calculate(current_value) / 2.0;
        final double high     = highpass.calculate(current_value);

        SmartDashboard.putNumber("Current", current_value);
        SmartDashboard.putNumber("Change10", change10);
        SmartDashboard.putNumber("Change5", change5);
        SmartDashboard.putNumber("Change3", change3);
        SmartDashboard.putNumber("Change2", change2);
        SmartDashboard.putNumber("Highpass", high);
        SmartDashboard.putBoolean("Ball Ejected", ball_ejected.compute(Math.abs(high) > 0.1));

        // Briefly pressing 'shoot' causes indication for 2 seconds
        SmartDashboard.putBoolean("Keep On", keep_on.compute(OperatorInterface.doShoot()));

        // Send out on each period to get better plots
        NetworkTableInstance.getDefault().flush();
    }
}
