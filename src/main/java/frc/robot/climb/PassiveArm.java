// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Passive arm which rotates up/down
 * 
 *  Both the left and right passive arm are
 *  controlled by the same solenoid,
 *  so to the software it looks like one arm
 */
public class PassiveArm extends SubsystemBase
{
    private Solenoid rotator = new Solenoid(RobotMap.PCM_TYPE, RobotMap.ARM_ROTATOR);

    public PassiveArm()
    {
        reset();
    }

    /** Reset to default position, all "up" */
    public void reset()
    {
        rotator.set(false);
    }

    /** @param down Rotate arm down? Otherwise up. */
    public void setAngle(final boolean down)
    {
        rotator.set(down);
    }

    /** @return Is arm down? */
    public boolean getAngle()
    {
        return rotator.get();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Arm Down", rotator.get());
    }
}
