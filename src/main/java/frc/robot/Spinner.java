// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Spinner
{
    private final WPI_TalonFX primary = new WPI_TalonFX(RobotMap.PRIMARY_SPINNER);
    private final WPI_TalonFX secondary = new WPI_TalonFX(RobotMap.SECONDARY_SPINNER);
    
    public Spinner()
    {
        secondary.setInverted(true); 
        secondary.follow(primary); 
    }

    public void setSpeed(double speed)
    {
        primary.set(speed);
    }
} 
