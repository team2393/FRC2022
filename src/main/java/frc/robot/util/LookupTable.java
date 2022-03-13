// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/** Lookup Table
 * 
 *  Performs linear interpolation between values in table
 */
public class LookupTable
{
    private static class Entry
    {
        final double position, speed;

        Entry(final double position, final double speed)
        {
            this.position = position;
            this.speed = speed;
        }
    }
    private final List<Entry> table = new ArrayList<>();
    
    /** @param position Position and ..
     *  @param speed .. speed to add to table
     */
    public void add(final double position, final double speed)
    {
        table.add(new Entry(position, speed));
        // Table must be sorted by position
        table.sort((a, b) -> Double.compare(a.position, b.position));
    }

    public double lookup(final double pos)
    {
        // Binary search
        int low = 0, high = table.size()-1;
        // Is position outside of x range?
        if (pos <= table.get(low).position)
            return table.get(low).speed;
        if (pos >= table.get(high).position)
            return table.get(high).speed;
        int i;
        do
        {   // Binary search: Find middle index
            i = (low + high) / 2;
            // Lucky, exact match?
            if (table.get(i).position == pos)
                return table.get(i).speed;
            else if (pos < table.get(i).position)
            {   // pos must be in the lower half
                high = i-1;
            }
            else
            {   // pos must be in upper half
                low = i + 1;
            }
        }
        while (low < high);
        // Interpolate
        return table.get(i).speed
            + (pos - table.get(i).position) * (table.get(i+1).speed - table.get(i).speed)
                                            / (table.get(i+1).position - table.get(i).position);
    }
 
    public static void main(String[] args)
    {
        // Example for lookup of spinner speeds for distance
        final LookupTable speeds = new LookupTable();
        speeds.add(50, 60);
        speeds.add(100, 65);
        speeds.add(150, 70);
        
        System.out.println(49 + " -> " + speeds.lookup(49));
        System.out.println(51 + " -> " + speeds.lookup(51));
        System.out.println(75 + " -> " + speeds.lookup(75));
        System.out.println(80 + " -> " + speeds.lookup(80));
        System.out.println(101 + " -> " + speeds.lookup(101));
        System.out.println(149 + " -> " + speeds.lookup(149));
        System.out.println(150 + " -> " + speeds.lookup(150));
        System.out.println(160 + " -> " + speeds.lookup(160));
    }
}