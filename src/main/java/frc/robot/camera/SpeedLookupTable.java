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
    // Distance [ty], Spinner Speed [rps]
           10.00, 64,
            3.15, 70,
           -0.70, 70,
           -4.50, 76,
           -6.3,  82,
           -6.60, 84,
           -9.65, 90);
    }
}
