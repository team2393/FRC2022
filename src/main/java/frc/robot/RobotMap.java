package frc.robot;

/** Information about anything that's plugged into the roboRio */
public class RobotMap
{
    // ** Power Distribution Panel **
    //
    // * 40 Amp connectors
    // 0 Drive motor                 15 Drive motor
    // 1 Drive motor                 14 Drive motor
    // 2                             13 
    // 3                             12   
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
    // 1) Some sensor?
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
    public final static int PRIMARY_LEFT_DRIVE = 1;
    public final static int SECONDARY_LEFT_DRIVE = 2;
    public final static int PRIMARY_RIGHT_DRIVE = 3;
    public final static int SECONDARY_RIGHT_DRIVE = 4;
}
