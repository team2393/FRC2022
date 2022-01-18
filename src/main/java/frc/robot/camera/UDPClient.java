/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020-2022 FIRST Team 2393. All Rights Reserved.              */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.camera;

import java.net.InetSocketAddress;
import java.net.StandardProtocolFamily;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

/** Receive vision info via UDP */
public class UDPClient
{
    private final DatagramChannel udp;
    private final ByteBuffer buffer = ByteBuffer.allocate(VisionData.BYTE_SIZE);

    /** Create client on default UDP port */
    public UDPClient() throws Exception
    {
        this(VisionData.UDP_PORT);
    }

    public UDPClient(final int port) throws Exception
    {
        udp = DatagramChannel.open(StandardProtocolFamily.INET);
        udp.configureBlocking(true);
        udp.socket().setBroadcast(true);
        udp.socket().setReuseAddress(true);
        udp.bind(new InetSocketAddress(port));
        System.out.println("UDP Client listening on " + udp.getLocalAddress());
    }

    /** Wait for an update from camera
     * 
     *  @return Data received from camera
     *  @throws Exception on error
     */
    public VisionData read() throws Exception
    {
        final VisionData data = new VisionData();

        // Read <whatever> into buffer
        // (blocks until we receive something)
        buffer.clear();
        udp.receive(buffer);

        // Assume that the buffer now contains vision data
        buffer.flip();
        data.decode(buffer);
        return data;
    }

    /** Test/demo 
     *  @throws Exception
     */
    public static void main(String[] args) throws Exception
    {
        System.out.println("Running UDP client, stop via 'Ctrl-c'");
        final UDPClient client = new UDPClient();
        final long start = System.currentTimeMillis();
        while (true)
        {
            final VisionData data = client.read();
            System.out.println((data.millisec - start) + " " + data);
        }    
    }
}
