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
    /** Class that holds one { position, speed } data point */
    public static class Entry
    {
        final public double position, speed, hood, deviation;

        Entry(final double position, final double speed,
              final double hood, final double deviation)
        {
            this.position = position;
            this.speed = speed;
            this.hood = hood;
            this.deviation = deviation;
        }

        @Override
        public String toString()
        {
            return String.format("Pos %f: Speed %f, Hood %f, Deviation %f",
                                 position, speed, hood, deviation);
        }
    }

    /** Table of data points */
    private final List<Entry> table = new ArrayList<>();

    public LookupTable(final double... pos_speed)
    {
        if (pos_speed.length % 4 != 0)
            throw new IllegalArgumentException("Need list of { pos, speed, hood, deviation}, {pos, speed, ...");
        for (int i = 0;  i < pos_speed.length;  i += 4)
            table.add(new Entry(pos_speed[i], pos_speed[i+1], pos_speed[i+2], pos_speed[i+3]));
        // Table must be sorted by position
        table.sort((a, b) -> Double.compare(a.position, b.position));
    }
    
    /** @param pos Position
     *  @return Speed for that position
     */
    public Entry lookup(final double pos)
    {
        final int n = table.size();
        // Is position outside of table's position range?
        if (pos <= table.get(0).position)
            return table.get(0);
        if (pos >= table.get(n-1).position)
            return table.get(n-1);
        // Binary search starting with left, right set to complete table
        // https://en.wikipedia.org/wiki/Binary_search_algorithm#Procedure_for_finding_the_leftmost_element
        int l = 0, r = n;
        while (l < r)
        {   // Binary search: Find middle index, rounding down(!)
            final int m = (l + r) / 2;
            if (table.get(m).position < pos)
                l = m+1;   // pos must be in upper half
            else
                r = m;     // pos must be in lower half (or exact match)
        }
        // For an exact match, [l] is that element
        if (table.get(l).position == pos)
            return table.get(l);
        // Otherwise l points to the next larger element,
        // so pos is between element [l-1] and [l].
        // Interpolate between those two points
        double slope = (table.get(l).speed    - table.get(l-1).speed)   /
                       (table.get(l).position - table.get(l-1).position);
        final double speed = table.get(l-1).speed + (pos - table.get(l-1).position) * slope;

        slope = (table.get(l).hood     - table.get(l-1).hood)   /
                (table.get(l).position - table.get(l-1).position);
        final double hood = table.get(l-1).hood + (pos - table.get(l-1).position) * slope;

        slope = (table.get(l).deviation - table.get(l-1).deviation)   /
                (table.get(l).position  - table.get(l-1).position);
        final double deviation = table.get(l-1).deviation + (pos - table.get(l-1).position) * slope;

        return new Entry(pos, speed, hood, deviation);
    }
 
    // Test/demo
    public static void main(String[] args)
    {
        // Example for lookup of spinner speeds for distance
        final LookupTable speeds = new LookupTable(30, 65, 0, 0,
                                                   20, 60, 0, 0,
                                                    0, 55, 0, 0,
                                                  -20, 60, 0, 0,
                                                  -30, 75, 0, 0);
        
        for (double d : new double[] { 40, 30, 25, 20, 10, 5, 0, -5, -10, -20, -30, -40})
            System.out.println(d + " -> " + speeds.lookup(d));
    }
}