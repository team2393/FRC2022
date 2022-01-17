// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

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
 */
public class ApplySettingsCommand extends CommandBase
{
    /** Name of one dashboard setting and its value */
    private class Setting
    {
        /** Name */
        final String name;

        /** Value, might be a number (double), a Boolean (true, false) or a string. */
        final Object value;

        Setting(final String the_name, final Object the_value)
        {
            name = the_name;
            value = the_value;
        }

        void apply()
        {
            if (value instanceof Double)
                SmartDashboard.putNumber(name, (Double) value);
            else if (value instanceof Boolean)
                SmartDashboard.putBoolean(name, (Boolean) value);
            else
                SmartDashboard.putString(name, (String) value);
            System.out.println("Setting " + name + " to " + value);
        }
    }

    /** List of settings that we read from the file */
    private final List<Setting> settings = new ArrayList<>();

    /** @param name Name of command on dashboard 
     *  @param filename Name of file within the deploy directory that lists "setting=value"
     */
    public ApplySettingsCommand(final String name, final String filename)
    {
        setName(name);
        readSettings(filename);
    }

    private void readSettings(final String filename) 
    {
        final File file = new File(Filesystem.getDeployDirectory(), filename);
        System.out.println("ApplySettingsCommand '" + getName() + "' reading " + file);
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
                // Ignore empty lines and comments
                if (line.isBlank()  ||  line.startsWith("#"))
                    continue;

                // Does the line contain "=" as in "SomeName = value"?
                final int sep = line.lastIndexOf("=");
                if (sep < 0)
                    continue;

                // Chop "SomeName  = value"  into "SomeName" and "value"
                final String name = line.substring(0, sep).trim();
                final String value = line.substring(sep+1).trim();
                System.out.println("Reading " + name + " = " + value);
    
                // Remember setting for when we later execute().
                // We have 'value' as a String.
                // But we likely need a Double or Boolean...
                try
                {   // Parse as number, which might fail if it's "true", "false" or other text
                    double number = Double.parseDouble(value);
                    settings.add(new Setting(name, number));
                }
                catch (NumberFormatException ex)
                {
                    // Not a number...
                    // Check if it's a boolean
                    if (value.equalsIgnoreCase("true"))
                        settings.add(new Setting(name, Boolean.TRUE));
                    else if (value.equalsIgnoreCase("false"))
                        settings.add(new Setting(name, Boolean.FALSE));
                    else // fall back to String
                    settings.add(new Setting(name, value));
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
        // Allow running this command even while disabled
        return true;
    }

    @Override
    public void execute()
    {
        System.out.println("Applying settings:");
        for (Setting setting : settings)
            setting.apply();
    }

    @Override
    public boolean isFinished()
    {
        // Once we 'execute()', we're done
        return true;
    }
}