/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020-2022 FIRST Team 2393. All Rights Reserved.              */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Various indicators based on digital LED strip */
public class LEDStrip
{
    // Adafruit 1m (30 LED) 'Neo Pixel'
    private static final int N = 30;
    private final AddressableLED strip = new AddressableLED(7);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(N);

    public LEDStrip()
    {
        strip.setLength(N);
        strip.start();
        idle();
    }

    /** Set all LEDs to same color
     *  @param r
     *  @param g
     *  @param b
     */
    public void setAll(final int r, final int g, final int b)
    {
        for (int i=0; i<N; ++i)
            buffer.setRGB(i, r, g, b);
        strip.setData(buffer);
    }

    /** 'Idle' type indicator */
    public void idle()
    {
        // Dark red
        setAll(50, 0, 0);
        strip.setData(buffer);
    }

    /** Millisecs for one rainbow cycle */
    private static final int rainbow_ms = 2000;

    /** Steps to advance hue for each 20 Hz update.
     *  One cycle means 180 degrees of hue.
     *  Split into rainbow_ms/1000 seconds,
     *  spread over 20 updates. 
     */
    private static final int rainbox_step = 180 / (rainbow_ms/1000) / 20;
    private int rainbox_start = 0;

    /** Cyling 'rainbow' pattern */
    public void rainbow()
    {
        for (int i=0; i<N; ++i)
            buffer.setHSV(i, (rainbox_start + (i * 180 / N)) % 180, 255, 128);
        rainbox_start = (rainbox_start + rainbox_step) % 180;

        strip.setData(buffer);
    }

    /** Number of LEDs used for the 'direction' indicator */
    private static final int pointer_size = 6;
    /** 'start' LED when indicator is in center */
    private static final int center_pos = (N - pointer_size) / 2;

    /** Indicate direction to a target
     *  @param direction -1 .. 0 .. 1 for left .. center .. right
     */
    public void indicateDirection(final double direction)
    {
        // 'on target', close to 0: ----------GGGGG----------
        // full left, -1          : GGGGGRRRRRRRRRR----------
        // somewhat left, -0.5    : ---GGGGGRRRRRRR----------

        // Compute 'start' of indicator, keeping it in valid range
        int start = center_pos + (int)Math.round(center_pos * direction);
        // Keep in valid range in case |direction| > 1 or rounding errors
        start = MathUtil.clamp(start, 0, N-pointer_size-1);
        final int end = Math.min(start + pointer_size, N-1);

        // Initial 'background' color
        int i;
        for (i=0; i<start; ++i)
            buffer.setRGB(i, 0, 0, 0);

        // Green start..end section
        for (i=start; i<end; ++i)
            buffer.setRGB(i, 0, 255, 0);

        // background to end
        for (i=end; i<N; ++i)
            buffer.setRGB(i, 0, 0, 0);

        // Potential red region from center to start (direction > 0)
        for (i=center_pos; i<start; ++i)
            buffer.setRGB(i, 255, 0, 0);

        // Potential red region from right end to center (direction < 0)
        for (i=end; i<center_pos+pointer_size; ++i)
            buffer.setRGB(i, 255, 0, 0);

        strip.setData(buffer);
    }

    public void bluewhite()
    {
        final int section = 3; // N must devide into sections!
        boolean phase = (System.currentTimeMillis() / 300) %2 == 1;

        int i = 0;
        while (i<N)
        {
            for (int s=0; s<section; ++s)
                if (phase)
                    buffer.setRGB(i++, 200, 200, 200);
                else
                    buffer.setRGB(i++, 0, 0, 255);
            phase = ! phase;
        }
        strip.setData(buffer);
    }

    public void oscillate()
    {
        final int size = 5;
        final double phase = 2*Math.PI * System.currentTimeMillis() / 1000;
        final int pos = N/2 + (int) ((N/2 - size + 1) * Math.sin(phase));

        for (int i=0; i<N; ++i)
            buffer.setRGB(i, 0, 0, 10);

        int start, end;
        if (pos > N/2)
        {
            start = MathUtil.clamp(pos, N/2, N);
            end = MathUtil.clamp(pos + size, N/2, N);
        }
        else
        {
            start = MathUtil.clamp(pos-size, 0, N/2);
            end = MathUtil.clamp(pos, 0, N/2);
        }
        for (int i=start; i<end; ++i)
            buffer.setRGB(i, 255, 255, 10);

        strip.setData(buffer);
    }
}
