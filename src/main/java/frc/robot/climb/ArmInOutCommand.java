package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmInOutCommand extends CommandBase
    {
        private final Climber climber;
        private final boolean in_or_out;

        public ArmInOutCommand(final Climber climber, final boolean in_or_out)
        {
            this.climber = climber;
            this.in_or_out = in_or_out;
        }
    
        @Override
        public void initialize() 
        {
            climber.setAngle(in_or_out);
        }
    
        @Override
        public boolean isFinished() 
        {
            return true;
        }
    }       
    
    
