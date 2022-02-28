// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.climb;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;

/** Set Climber to fixed extension
 *  (but allow editing that value via dashboard)
 */
public class SetClimberExtensionCommand extends CommandBase
{
    private final Climber climber;
    private final NetworkTableEntry setting;

    public SetClimberExtensionCommand(final Climber climber, final String setting_name, final double extension)
    {
        this.climber = climber;
        addRequirements(climber);

        // Publish initial/default value on dashboard ..
        setting = SmartDashboard.getEntry(setting_name);
        setting.setDefaultDouble(extension);
    }

    @Override
    public void execute()
    {
        // .. and use the value that's now on the DB
        climber.setExtension(setting.getDouble(0.0));

        if (OperatorInterface.toggleArmAngle())
            climber.setAngle(! climber.getAngle());
    }
}
