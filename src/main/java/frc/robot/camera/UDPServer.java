/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020-2022 FIRST Team 2393. All Rights Reserved.              */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.camera;

import java.net.InetSocketAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Send vision data updates via UDP */
public class UDPServer
{
    private final DatagramChannel udp;
    private final ByteBuffer buffer = ByteBuffer.allocate(VisionData.BYTE_SIZE);
    private final List<InetSocketAddress> broadcasts = new ArrayList<>();

    /** Create server on default port */
    public UDPServer() throws Exception
    {
        this(VisionData.UDP_PORT);
    }

    public UDPServer(final int port) throws Exception
    {
        // Create a 'socket' that can use broadcasts
        udp = DatagramChannel.open(StandardProtocolFamily.INET);
        udp.configureBlocking(true);
        udp.socket().setBroadcast(true);
        udp.socket().setReuseAddress(true);

        // Find all network interfaces that support broadcast
        for (NetworkInterface iface : Collections.list(NetworkInterface.getNetworkInterfaces()))
            if (iface.isUp())
                for (InterfaceAddress addr : iface.getInterfaceAddresses())
                    if (addr.getBroadcast() != null)
                        broadcasts.add(new InetSocketAddress(addr.getBroadcast(), port));

        final InetSocketAddress team_net = new InetSocketAddress("10.23.93.255", port);
        if (! broadcasts.contains(team_net))
            broadcasts.add(team_net);

        System.out.println("UDP Server broadcasting to " + broadcasts);
    }

    /** @param data Data to send via UDP */
    public void send(final VisionData data)
    {
        // Place number in byte buffer
        try
        {
            buffer.clear();
            data.encode(buffer);

            // Send as broadcast
            for (InetSocketAddress addr : broadcasts)
            {
                buffer.flip();
                udp.send(buffer, addr);
            }
        }
        catch(Exception ex)
        {
            ex.printStackTrace();
        }
    }

    /** Test/demo 
     *  @throws Exception
     */
    public static void main(String[] args) throws Exception
    {
        System.out.println("Running UDP server, stop via 'Ctrl-C'");
        final UDPServer server = new UDPServer();
        final VisionData data = new VisionData();
        data.clear();
        while (true)
        {
            Thread.sleep(1000);
            
            data.distance += 1;
            data.direction += 2;
            server.send(data);
        }   
    }
}
