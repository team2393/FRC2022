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
    // * Ports with 40 Amp fuses
    //  0 Drive motor 1/4             19 Drive motor 3/4
    //  1 Drive motor 2/4             18 Drive motor 4/4
    //  2 Spinner motor 1/2           17 Spinner motor 2/2
    //  3 Intake motor 1/2            16 Intake motor 2/2
    //  4 Left arm                    15 Right Arm
    //
    //  * 20 Amp ports
    //  5 Conveyor motor              14 Feeder motor
    //  6 Pneumatic Hub               13 
    //  7 Limelight                   12
    //  8                             11
    //
    //  * 20 Amp ports
    //  9 VRM for sensors             10
    //
    // Port 20, 10 Amp: Radio Power Module
    // Port 21, 10 Amp: RoboRIO
    // Port 23 (switchable)  Camera LED ring
    //
    // VRM 12V, 500mA
    // 1) Conveyor sensor
    // 2) Feeder Sensor
    //
    // VRM 5V, 500mA
    // 1)
    // 2)
    //
    // VRM 5V, 2A
    // 1) Raspberry Pi
    // 2) Ethernet hub

    // Motors connected via CAN bus
    // Falcon unless otherwise specified
    public final static int PRIMARY_LEFT_DRIVE = 4;
    public final static int SECONDARY_LEFT_DRIVE = 2;
    public final static int PRIMARY_RIGHT_DRIVE = 3;
    public final static int SECONDARY_RIGHT_DRIVE = 1;
    public final static int PRIMARY_SPINNER = 5;
    public final static int SECONDARY_SPINNER = 6;
    public static final int LEFT_ARM_EXTENDER = 7;
    public static final int RIGHT_ARM_EXTENDER = 8;
    public final static int LEFT_INTAKE = 9;   // TalonFX
    public final static int RIGHT_INTAKE = 10; // TalonFX
    public final static int CONVEYOR = 11;
    public final static int FEEDER = 12;
    
    // Solenoids
    // Are we using original PCM (of which we have many)
    // or new REV PH (more channels, but only one)?
    public final static PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.REVPH;
    public static final int ARM_ROTATOR = 0; //A
    public final static int SHOOTER_ANGLE = 1; //B
    public final static int GEAR_SHIFTER = 2; //C
    public final static int INTAKE_ARM = 3; //D

    // Digital I/O
    public final static int CONVEYOR_SENSOR = 0; //  Using CargoSensor ?
    public final static int FEEDER_SENSOR = 1;
    public final static int EJECTION_SENSOR = 2;
    public static final int LEFT_ARM_RETRACTED = 3;
    public static final int RIGHT_ARM_RETRACTED = 4;

    // PMW Channels
    public final static int LED_STRING = 5;
}
