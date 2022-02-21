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
 *  so that's what we use here as well!
 */
public class Pneumatics extends SubsystemBase
{
    private final PneumaticHub hub = new PneumaticHub();
    private final NetworkTableEntry nt_pressure;

    public Pneumatics()
    {
        // USE ANALOG SENSOR,
        // and turn compressor on below 70 psi, stop when reaching 120 psi
        hub.enableCompressorAnalog(70, 120);
        nt_pressure = SmartDashboard.getEntry("Pressure");
    }
    
    @Override
    public void periodic()
    {
        nt_pressure.setDouble(hub.getPressure(1));
    }
}
