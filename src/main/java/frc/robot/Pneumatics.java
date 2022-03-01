// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Pneumatics Support
 * 
 *  NOTE:
 *  https://docs.revrobotics.com/rev-11-1852/pneumatic-hub-getting-started/wiring-the-pneumatic-hub
 *  suggests plugging the analog sensor into input 1, not 0,
 *  but #812 says
 *  "The analog output of the sensor must be connected directly to analog input 0 of the PH
 *   (with firmware version 22.0.2 or newer) controlling the compressor"
 */
public class Pneumatics extends SubsystemBase
{
    private final PneumaticHub hub = new PneumaticHub();
    private final NetworkTableEntry nt_pressure;

    public Pneumatics()
    {
        // USE ANALOG SENSOR,
        // and turn compressor on below 100 psi, stop when reaching 120 psi
        hub.enableCompressorAnalog(85, 120);
        nt_pressure = SmartDashboard.getEntry("Pressure");
    }
    
    @Override
    public void periodic()
    {
        // Read analog input 0
        nt_pressure.setDouble(hub.getPressure(0));
    }
}
