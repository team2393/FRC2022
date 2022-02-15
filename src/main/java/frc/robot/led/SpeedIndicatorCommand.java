package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;

public class SpeedIndicatorCommand extends CommandBase
{
    private LEDStrip strip;

    public SpeedIndicatorCommand(LEDStrip strip)
    {
        this.strip = strip;
    }

    @Override
    public void execute()
    {
        double speed = OperatorInterface.getSpeed();

        strip.setAll(Color.kBlack);

        if (speed > 0)
        {
            int count = (int) (15*speed);
            for (int i=15;  i < 15+count; ++i)
                strip.set(i, Color.kGreen);
        }
        else if (speed < 0)
        {
            int count = (int) (-15*speed);
            for (int i=15;  i > 15-count; --i)
                strip.set(i, Color.kRed);
        }


    }
}
