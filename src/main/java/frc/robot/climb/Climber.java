// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Climber: Left and right arms, ... */
public class Climber extends SubsystemBase
{
    private final ActiveArm left_active = new ActiveArm(RobotMap.LEFT_ARM_EXTENDER, RobotMap.LEFT_ARM_RETRACTED);    
    private final ActiveArm right_active = new ActiveArm(RobotMap.RIGHT_ARM_EXTENDER, RobotMap.RIGHT_ARM_RETRACTED);    
    private final PassiveArm passive = new PassiveArm();

    public void reset()
    {
        passive.reset();
        left_active.reset();
        right_active.reset();
    }

    /** @param down Rotate arm down? Otherwise up. */
    public void setAngle(final boolean down)
    {
        passive.setAngle(down);
    }
    
    /** @return Is arm down? */
    public boolean getAngle()
    {
        return passive.getAngle();
    }

    /** Perform homing operation
     * 
     *  Move both extender "in" until they hit the limit,
     *  then reset encoders to zero.
     * 
     *  @return Are we done, both fully retracted?
     */
    public boolean homing()
    {
        final boolean left_homed = left_active.homing();
        final boolean right_homed = right_active.homing();
        return left_homed && right_homed;
    }

    /** @param voltage Entender voltage, positive for "out" */
    public void setExtenderVoltage(final double voltage)
    {
        left_active.setExtenderVoltage(voltage);
        right_active.setExtenderVoltage(voltage);
    }

    /** @param extension Desired extension in meters */
    public void setExtension(final double extension)
    {
        left_active.setExtension(extension);
        right_active.setExtension(extension);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Left Arm Extension", left_active.getExtension());
        SmartDashboard.putNumber("Right Arm Extension", right_active.getExtension());
    }
}
