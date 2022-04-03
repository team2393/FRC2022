// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static java.lang.Math.sqrt;

/** Basic exploration of serve helpers in WPILib */
public class Swerving3
{
    /** Distance of each module from center of robot */
    private static final double module_distance = sqrt(0.5*0.5 + 0.5*0.5);

    /** Swerve modules #0, 1, 2 on points of triangle
     * 
     *      Y
     *     /|\
     *      |
     *      |
     * 2    |
     *      |
     * -----*-----0--->  X
     *      |
     * 1    |
     */
    private static final Translation2d[] modules = new Translation2d[]
    {
        new Translation2d(module_distance, Rotation2d.fromDegrees(0)),
        new Translation2d(module_distance, Rotation2d.fromDegrees(-120)),
        new Translation2d(module_distance, Rotation2d.fromDegrees(+120)),
    };

    private static final int N = modules.length;

    /** Transform robot movement into swerve module speeds and vice versa */
    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modules);

    /** Track current position */
    private static final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0.0));

    // Once everything 'works', would use this to follow a trajectory:
    // SwerveControllerCommand c;

    private static void print(final SwerveModuleState[] module_states)
    {
        for (int i=0; i<N; ++i)
            System.out.format("#%d: %6.2f m/s @%6.1f deg\n",
                              i,
                              module_states[i].speedMetersPerSecond,
                              module_states[i].angle.getDegrees());
    }

    private static void print(final Pose2d pose)
    {
        System.out.format("X=%6.2f m, Y=%6.2f m, Heading %6.1f deg\n",
                          pose.getX(),
                          pose.getY(),
                          pose.getRotation().getDegrees());
    }

    public static void main(String[] args)
    {
        System.out.println("Module locations:");
        for (int i=0; i<N; ++i)
            System.out.format("#%d: X=%5.2f m, Y=%5.2f m\n", i, modules[i].getX(), modules[i].getY());

        System.out.print("\nInitial position: ");
        print(odometry.getPoseMeters());

        System.out.println("\nDrive straight along X at 1 m/s:");
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));
        print(states);

        // First call with  time = 0 sec initializes odometry
        double now = 0.0;
        odometry.updateWithTime(now, Rotation2d.fromDegrees(0), states);
        // Simulate moving as per 'states' for one second
        now += 1.0;
        odometry.updateWithTime(now, Rotation2d.fromDegrees(0), states);
        System.out.format("Position at %.1f s: ", now);
        print(odometry.getPoseMeters());

        System.out.println("\nDrive straight along Y at 1 m/s:");
        states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 1.0, 0.0));
        print(states);
        now += 1.0;
        odometry.updateWithTime(now, Rotation2d.fromDegrees(0), states);
        System.out.format("Position at %.1f s: ", now);
        print(odometry.getPoseMeters());

        System.out.println("\nRotate in place at 10 deg/sec:");
        print(kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, Math.toRadians(10))));
        System.out.println("\nRotate in place at -10 deg/sec:");
        print(kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, Math.toRadians(-10))));
        // Not updating odometry since we'd be in same place..

        System.out.println("\nDrive straight along X at 1 m/s while rotating at 1 deg/sec:");
        states = kinematics.toSwerveModuleStates(new ChassisSpeeds(1.0, 0.0, Math.toRadians(10)));
        print(states);
        now += 1.0;
        odometry.updateWithTime(now, Rotation2d.fromDegrees(10), states);
        System.out.format("Position at %.1f s: ", now);
        print(odometry.getPoseMeters());
    }
}
