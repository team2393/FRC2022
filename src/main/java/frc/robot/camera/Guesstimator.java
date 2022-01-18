// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.camera;

import frc.robot.util.CircularBuffer;

/** Guess camera data
 * 
 *  Camera runs at 30 frames/sec, so at best we get 30 updates/sec
 *  on the direction to the target, often less frequently.
 *  The robot runs at 50 Hz, so more often than not it looks at old
 *  data.
 * 
 *  By remembering the last two updates that we received
 *  from the camera, we can try to guess what the next update
 *  will be.
 *  We draw a line from the last two  data points over time to 'now'
 *  to have a reasonable guess for the current value,
 *  assuming that the robot keeps moving the same way.
 */
public class Guesstimator 
{
    /** If the oldest camera update is half a second old, forgettaboutit */
    private static final long TOO_OLD_MS = 500;

    /** Ring buffer for the last two camera updates */
    private CircularBuffer<VisionData> updates = new CircularBuffer<>(2);

    /** @param update Latest camera update */
    public void addData(final VisionData update)
    {
       updates.add(update);
    }

    /** Extrapolate from two data points onto another time 
     * 
     *  @param t1 Time when first data point was taken
     *  @param v1 Value of first point
     *  @param t2 Time when second data point was taken
     *  @param v2 Value of second point
     *  @param t Time for which we want to estimate the value
     *  @return Estimated value
     */
    private double extrapolate(double t1, double v1, double t2, double v2, double t)
    {
        // Compute line through points (t1, v1)  and  (t2, v2),
        // i.e. x axis is 'time', and y axis for value

        // 1) Compute slope
        final double slope = (v2 - v1) / (t2 - t1);

        // 2) Compute value axis intersection, either via
        //    v1 = v0 + slope * t1  or
        //    v2 = v0 + slope * t2
        final double v0 = v1 - slope * t1;

        // Now get value at requested time
        return v0 + slope * t;
    }

    /** Try to predict what the camera should see right now,
     *  based on the last two updates that we received.
     * 
     *  @return Estimated {@link CameraData}, set to zero direction & distance if we have no clue
     */
    public VisionData guesstimate()
    {
        final VisionData result = new VisionData();
        final long now = result.millisec = System.currentTimeMillis();

        // Do we have two updates?
        if (updates.size() < 2)
            return result;

        final VisionData oldest = updates.get(0);
        final VisionData last = updates.get(1);

        // Are they fresh enough?
        if (oldest.millisec < now - TOO_OLD_MS)
            return result;

        // Draw line through the old points, then get value 'now'
        if (oldest.millisec == last.millisec)
        {
            // This would result in division-by-zero when computing the slope..
            System.out.println("Guesstimator: Time is not progressing");
            return result;
        }
        result.direction = (int) extrapolate(oldest.millisec, oldest.direction,
                                             last.millisec, last.direction,
                                             now);
        result.distance = (int) extrapolate(oldest.millisec, oldest.distance,
                                            last.millisec, last.distance,
                                            now);
        return result;
    }

    public static void main(String[] args)
    {
        Guesstimator guesstimator = new Guesstimator();

        // Simulate camera updates every 1000/30 = 33 ms
        final long now = System.currentTimeMillis();
        guesstimator.addData(new VisionData(now-66, 100,  0));  
        guesstimator.addData(new VisionData(now-33,  90, 10));

        // Guess what the next update would be if the same trend continued
        VisionData guess = guesstimator.guesstimate();
        System.out.println(guess);
        System.out.println("^^ Should be close to Direction 80, distance 20 ^^");
    }
}
