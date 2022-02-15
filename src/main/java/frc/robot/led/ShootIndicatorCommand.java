package frc.robot.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootIndicatorCommand extends CommandBase
{
    private LEDStrip strip;
    private Color color;
    private int position;

    public ShootIndicatorCommand(LEDStrip strip, Color color)
    {
        this.strip = strip;
        this.color = color;
        addRequirements(strip);
    }
    
    @Override
    public void initialize()
    {
        strip.setAll(Color.kBlack);
        position = 0;
    }

    @Override
    public void execute()
    {
        strip.set(position, Color.kBlack);
        strip.set(position + 1, Color.kBlack);
        strip.set(position + 2, Color.kBlack);
        
        position = position + 2;
        if (position > 27)
            position = 0;
            
        strip.set(position, color);
        strip.set(position + 1, color);
        strip.set(position + 2, color);
    }
}
