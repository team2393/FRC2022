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
    private static class Entry
    {
        final double position, speed;

        Entry(final double position, final double speed)
        {
            this.position = position;
            this.speed = speed;
        }
    }

    /** Table of data points */
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

    /** @param pos Position
     *  @return Speed for that position
     */
    public double lookup(final double pos)
    {
        final int n = table.size();
        // Is position outside of table's position range?
        if (pos <= table.get(0).position)
            return table.get(0).speed;
        if (pos >= table.get(n-1).position)
            return table.get(n-1).speed;
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
            return table.get(l).speed;
        // Otherwise l points to the next larger element,
        // so pos is between element [l-1] and [l].
        // Interpolate between those two points
        final double slope = (table.get(l).speed    - table.get(l-1).speed)   /
                             (table.get(l).position - table.get(l-1).position);
        return table.get(l-1).speed + (pos - table.get(l-1).position) * slope;
    }
 
    // Test/demo
    public static void main(String[] args)
    {
        // Example for lookup of spinner speeds for distance
        final LookupTable speeds = new LookupTable();
        speeds.add(5, 5);
        speeds.add(100, 100);
        speeds.add(150, 1500);
        
        System.out.println(4 + " -> " + speeds.lookup(4));
        System.out.println(5 + " -> " + speeds.lookup(5));
        System.out.println(6 + " -> " + speeds.lookup(6));
        System.out.println(75 + " -> " + speeds.lookup(75));
        System.out.println(80 + " -> " + speeds.lookup(80));
        System.out.println(100 + " -> " + speeds.lookup(100));
        System.out.println(101 + " -> " + speeds.lookup(101));
        System.out.println(120 + " -> " + speeds.lookup(120));
        System.out.println(149 + " -> " + speeds.lookup(149));
        System.out.println(150 + " -> " + speeds.lookup(150));
        System.out.println(160 + " -> " + speeds.lookup(160));
    }
}