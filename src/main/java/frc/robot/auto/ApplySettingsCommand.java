// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that applies settings from file to network tables
 * 
 *  File must be in the 'deploy' folder on the RoboRIO.
 *  Lines with "# comments" are ignored, so are empty lines.
 *  Lines with format "SomeName = Value" are remembered
 *  and values are applied to those named settings whenever the
 *  command gets executed.
 * 
 *  Value might be a number (double), a Boolean ("true", "false")
 *  or a string.
 */
public class ApplySettingsCommand extends CommandBase
{
    /** Name of setting and value (Double, Boolean or String) */
    private final Map<String, Object> settings = new HashMap<>();

    /** @param name Name of command on dashboard 
     *  @param filename Name of file within the deploy directory that lists "setting=value"
     */
    public ApplySettingsCommand(final String name, final String filename)
    {
        setName(name);

        final File file = new File(Filesystem.getDeployDirectory(), filename);
        System.out.println("ApplySettingsCommand '" + name + "' reading " + file);
        try
        (
            final BufferedReader reader = new BufferedReader(new FileReader(file))
        )
        {
            // Read file line by line
            for (String line = reader.readLine();
                 line != null;
                 line = reader.readLine())
            {
                // Remove leading and trailing spaces
                line = line.trim();
                // Ignore comments
                if (line.isBlank()  ||  line.startsWith("#"))
                    continue;

                // Does the line contain a "=" as in "SomeName = value"?
                final int sep = line.lastIndexOf("=");
                if (sep < 0)
                    continue;

                // Chop "SomeName  = value"  into "SomeName" and "value"
                final String setting = line.substring(0, sep).trim();
                final String value = line.substring(sep+1).trim();
                System.out.println("Reading " + setting + " = " + value);
    
                // Remember setting and value for later when we execute().
                // We have 'value' as a String.
                // But we likely need a Double or Boolean...
                try
                {   // Parse as number, which might fail if it's "true", "false" or other text
                    double number = Double.parseDouble(value);
                    settings.put(setting, number);
                }
                catch (NumberFormatException ex)
                {
                    // Not a number...
                    // Check if it's a boolean
                    if (value.equalsIgnoreCase("true"))
                        settings.put(setting, Boolean.TRUE);
                    else if (value.equalsIgnoreCase("false"))
                        settings.put(setting, Boolean.FALSE);
                    else // fall back to String
                        settings.put(setting, value);
                }
            }
        }
        catch (Exception ex)
        {
            System.out.println("Cannot read " + file);
            ex.printStackTrace();
        }
    }

    @Override
    public boolean runsWhenDisabled()
    {
        return true;
    }

    
    @Override
    public void execute()
    {
        // Apply the settings!
        System.out.println("Applying settings:");
        for (String setting : settings.keySet())
        {
            Object value = settings.get(setting);
            if (value instanceof Double)
                SmartDashboard.putNumber(setting, (Double) value);
            else if (value instanceof Boolean)
                SmartDashboard.putBoolean(setting, (Boolean) value);
            else
                SmartDashboard.putString(setting, (String) value);
            System.out.println("Setting " + setting + " to " + value);
        }
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}