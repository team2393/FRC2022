/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST Team 2393. All Rights Reserved.              */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.camera;

import java.nio.ByteBuffer;

/** Data that we get from the camera */
public class VisionData
{
    /** Size of data in buffer bytes */
    public static final int BYTE_SIZE = Integer.BYTES*2;

    /** UDP port used to communicate the vision data
     * 
     *  2022 game manual R704: UDP/TCP 5800-5810 open for Team use
     */
    public static final int UDP_PORT = 5801;

    /** Direction to target and distance
     * 
     *  .. in units of pixels relative to center of camera image
     */
    public int direction, distance;

    /** Time in millisec when data was received (not used when sending) */
    public long millisec;

    /** Clear the data
     * 
     *  Set distance and direction to "safe" values.
     *  When robot believes the target is in the center of the camera image,
     *  it should stay put, not move.
     */
    public void clear()
    {
        direction = distance = 0;
    }

    /** @param buffer Buffer into which the data is written */
    public void encode(final ByteBuffer buffer)
    {
        buffer.putInt(direction);
        buffer.putInt(distance);
    }

    /** @param buffer Buffer from which data is read */
    public void decode(final ByteBuffer buffer)
    {
        millisec = System.currentTimeMillis();
        direction = buffer.getInt();
        distance = buffer.getInt();
    }

    @Override
    public String toString()
    {
        return "Direction " + direction + ", Distance " + distance;
    }


    /** Test/demo */
    public static void main(String[] args)
    {
        final ByteBuffer buffer = ByteBuffer.allocate(VisionData.BYTE_SIZE);
        final VisionData data = new VisionData();

        data.direction = 42;
        data.distance = -20;
        data.encode(buffer);
        System.out.println("Encoded: " + data);

        data.clear();
        System.out.println("Cleared: " + data);

        buffer.flip();
        data.decode(buffer);
        System.out.println("Decoded: " + data);
    }
}