// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.camera;

import frc.robot.util.LookupTable;

/** Spinner speeds for various distances */
public class SpeedLookupTable extends LookupTable
{
    public SpeedLookupTable()
    {
        super(
    // Distance [ty], Spinner Speed [rps], Hood [mm], Deviation [deg]
           10.10, 35,  5, 0,
            7.2,  35, 40, 0,
            0.8,  35, 70, 0,
           -3.0,  38, 60, 0,
           -7.0,  42, 50, 0,
           -7.5,  41, 70, 0,
           -9.3,  40, 90, 0,
           -9.7,  42, 90, 0,
          -11.0,  44, 80, 0);
    }
}
