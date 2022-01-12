package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Spinner
{
    private final WPI_TalonFX primary = new WPI_TalonFX(RobotMap.PRIMARY_SPINNER);
    private final WPI_TalonFX secondary = new WPI_TalonFX(RobotMap.SECONDARY_SPINNER);
    
    public void Spinner()
    {
        secondary.setInverted(true); 
        secondary.follow(primary); 
    }

    public void setSpeed(double speed)
    {

        primary.set(speed);

    }

} 
