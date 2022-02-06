// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;
import static frc.robot.led.LEDStrip.N;

/** Command that indicates where we turn */
public class TurnIndicatorCommand extends CommandBase
{
    private final LEDStrip strip;

    /** Number of LEDs used for the 'direction' indicator */
    private static final int pointer_size = 6;
    /** 'start' LED when indicator is in center */
    private static final int center_pos = (N - pointer_size) / 2;    

    public TurnIndicatorCommand(final LEDStrip strip)
    {
        this.strip = strip;
        addRequirements(strip);
    }

    @Override
    public void execute()
    {
        final double direction = OperatorInterface.getRotation();

        // 'straight', close to 0: ----------GGGGG----------
        // full left, -1         : GGGGGRRRRRRRRRR----------
        // somewhat left, -0.5   : ---GGGGGRRRRRRR----------

        // Compute 'start' of indicator, keeping it in valid range
        int start = center_pos + (int)Math.round(center_pos * direction);
        // Keep in valid range in case |direction| > 1 or rounding errors
        start = MathUtil.clamp(start, 0, N-pointer_size-1);
        final int end = Math.min(start + pointer_size, N-1);

        // Initial 'background' color
        int i;
        for (i=0; i<start; ++i)
            strip.set(i, 0, 0, 0);

        // Green start..end section
        for (i=start; i<end; ++i)
            strip.set(i, 0, 255, 0);

        // background to end
        for (i=end; i<N; ++i)
            strip.set(i, 0, 0, 0);

        // Potential red region from center to start (direction > 0)
        for (i=center_pos; i<start; ++i)
            strip.set(i, 255, 0, 0);

        // Potential red region from right end to center (direction < 0)
        for (i=end; i<center_pos+pointer_size; ++i)
            strip.set(i, 255, 0, 0);
    }
}
