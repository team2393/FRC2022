// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Information about anything that's plugged into the roboRIO */
public class RobotMap
{
    // ** Power Distribution Panel **
    //
    // * 40 Amp connectors
    // 0 Drive motor                 15 Drive motor
    // 1 Drive motor                 14 Drive motor
    // 2 Spinner motor               13 Spinner motor
    // 3                             12 Feeder motor(s)? Conveyor motor(s)? Intake motor(s)?
    //
    // * Less-than-40 Amp ports
    // 4                             11 
    // 5                             10 
    // 6                              9 
    // 7                              8 
    //
    // PDP controller port -> RoboRIO
    // PDP PCM port -> PCM, compressor, solenoids
    //
    // PDP VRM port ->
    // VRM 12V, 2A
    // 1) Radio
    // 2) must not be used, see game manual!
    //
    // VRM 12V, 500mA
    // 1) Some ball sensor?
    // 2) 
    //
    // VRM 5V, 500mA
    // 1)
    // 2)
    //
    // VRM 5V, 2A
    // 1) Raspberry Pi power?
    // 2) Color LED strip power?

    // Motors connected via CAN bus
    public final static int PRIMARY_LEFT_DRIVE = 4;
    public final static int SECONDARY_LEFT_DRIVE = 2;
    public final static int PRIMARY_RIGHT_DRIVE = 3;
    public final static int SECONDARY_RIGHT_DRIVE = 1;
    public final static int PRIMARY_SPINNER = 5;
    public final static int SECONDARY_SPINNER = 6;
    public static final int LEFT_ARM_EXTENDER = 7;
    public static final int RIGHT_ARM_EXTENDER = 8;
    public final static int INTAKE = 9;
    public final static int CONVEYOR = 10;
    public final static int FEEDER = 11;
    
    // Solenoids
    // Are we using original PCM (of which we have many)
    // or new REV PH (more channels, but only one)?
    public final static PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
    public final static int INTAKE_ARM = 0;
    public final static int SHOOTER_ANGLE = 1;
    public static final int ARM_ROTATOR = 2;
    public final static int GEAR_SHIFTER = 6;

    // Digital I/O
    // public final static int CONVEYOR_SENSOR = 0;  Using CargoSensor
    public final static int FEEDER_SENSOR = 1;
    public final static int EJECTION_SENSOR = 2;
    public static final int LEFT_ARM_RETRACTED = 3;
    public static final int RIGHT_ARM_RETRACTED = 4;
}
