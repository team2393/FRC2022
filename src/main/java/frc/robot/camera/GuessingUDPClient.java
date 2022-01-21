// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.camera;

/** Thread that keeps reading from UDPClient, using Guesstimator */
public class GuessingUDPClient
{
    private final UDPClient client;
    private final Thread thread;
    // SYNC on access
    private final Guesstimator guesstimator = new Guesstimator();

    public GuessingUDPClient()
    {
        client = new UDPClient();
        thread = new Thread(this::receive);
        thread.setDaemon(true);
        thread.start();
    }

    private void receive()
    {
        long last_ms = 0;
        try
        {
            while (true)
            {
                // Wait for update from camera..
                final VisionData update = client.read();

                // Camera may send the same UDP packet multiple times,
                // so ignore packet that arrives just a few ms later
                if ( update.millisec - last_ms < 5)
                    continue;

                last_ms = update.millisec;
                // .. and add to guesstimator
                synchronized(guesstimator)
                {
                    guesstimator.addData(update);
                }
            }
        }
        catch (Exception ex)
        {
            ex.printStackTrace();
        }
        System.err.println("GuessingUDPClient quits");
    }

    /** @return Guesstimated camera data */
    public VisionData get()
    {
        synchronized (guesstimator)
        {
            return guesstimator.guesstimate();
        }
    }
}
