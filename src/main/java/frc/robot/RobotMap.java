// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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

    public final static int INTAKE = 7;
    public final static int CONVEYOR = 8;
    public final static int FEEDER = 9;


    // Solenoids
    public final static int INTAKE_ARM = 0;

    // Digital I/O
    public final static int  CONVEYOR_SENSOR = 0;
    public final static int  FEEDER_SENSOR = 1;
    public final static int  EJECTION_SENSOR = 2;

}
