// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Digital, individually addressable strip of LEDs */
public class LEDStrip extends SubsystemBase
{
    // Adafruit 1m (30 LED) 'Neo Pixel', shortened to 24

    /** Number of LEDs */
    public static final int N = 24;
    
    private final AddressableLED strip = new AddressableLED(RobotMap.LED_STRING);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(N);

    public LEDStrip()
    {
        strip.setLength(N);
        strip.start();
        setAll(Color.kBlack);
    }

    /** Set color of one LED
     *  @param index  0..(N-1)
     *  @param red 0..255
     *  @param green 0..255
     *  @param blue 0.255
     */
    public void set(final int index, final int red, final int green, final int blue)
    {
        buffer.setRGB(index, red, green, blue);
    }

    /** Set color of one LED
     *  @param index  0..(N-1)
     *  @param color Color
     */
    public void set(final int index, final Color color)
    {
        final Color8Bit c8 = new Color8Bit(color);
        set(index, c8.red, c8.green, c8.blue);
    }

    /** Set all LEDs to same color
     *  @param red 0..255
     *  @param green 0..255
     *  @param blue 0.255
     */
    public void setAll(final int red, final int green, final int blue)
    {
        for (int i=0; i<N; ++i)
            set(i, red, green, blue);
    }

    /** Set all LEDs to same color
     *  @param color The Color to use
     */
    public void setAll(final Color color)
    {
        for (int i=0; i<N; ++i)
            set(i, color);
    }

    @Override
    public void periodic()
    {
        // Send current LED color settings out to the strip
        strip.setData(buffer);
    }    
}
