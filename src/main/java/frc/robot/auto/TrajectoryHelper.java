// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.drivetrain.Drivetrain;

/** Helpers for dealing with trajectories */
public class TrajectoryHelper
{
    /** Create forward-moving trajectory from points
     * 
     *  Trajectory starts at X=0, Y=0 and Heading = 0.
     *  Given list of points must contain entries x, y, h,
     *  i.e., total length of x_y_h array must be a multiple of 3.
     * 
     *  @param x_y_z Sequence of points { X, Y, Heading }
     */
    public static Trajectory createTrajectory(final double... x_y_h)
    {
        return createTrajectory(true, x_y_h);
    }

    /** Create trajectory from points
     * 
     *  Trajectory starts at X=0, Y=0 and Heading = 0.
     *  Given list of points must contain entries x, y, h,
     *  i.e., total length of x_y_h array must be a multiple of 3.
     * 
     *  @param forward Are we driving forward?
     *  @param x_y_z Sequence of points { X, Y, Heading }
     */
    public static Trajectory createTrajectory(final boolean forward, final double... x_y_h)
    {
        if (x_y_h.length % 3 != 0)
            throw new IllegalArgumentException("List of { X, Y, Heading } contains " + x_y_h.length + " entries?!");

        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        for (int i=0; i<x_y_h.length; i += 3)
            waypoints.add(new Pose2d(x_y_h[i], x_y_h[i+1], Rotation2d.fromDegrees(x_y_h[i+2])));

        Drivetrain.trajectory_config.setReversed(! forward);
        return TrajectoryGenerator.generateTrajectory(waypoints, Drivetrain.trajectory_config);
    }

    /** @param trajectory Trajectory
     *  @return End state
     */
    public static State getStart(final Trajectory trajectory)
    {
        final List<State> states = trajectory.getStates();
        return states.get(0);
    }

    /** @param trajectory Trajectory
     *  @return End state
     */
    public static State getEnd(final Trajectory trajectory)
    {
        final List<State> states = trajectory.getStates();
        return states.get(states.size()-1);
    }

    /** @param trajectory Trajectory
     *  @return End pose
     */
    public static Pose2d getEndPose(final Trajectory trajectory)
    {
        return getEnd(trajectory).poseMeters;
    }
    public static double getMaxVelocity(final Trajectory trajectory)
    {
        double max_speed = 0.0;
        for (State state : trajectory.getStates())
        {
            final double speed = Math.abs(state.velocityMetersPerSecond);
            if (speed > max_speed)
            max_speed = speed;
        }
        return max_speed;
    }
  
    /** @param state One state along a Trajectory
     *  @return "X=..., Y=..., ..."
     */
    public static String getInfo(final State state)
    {
        return String.format("X=%.2f m, Y=%.2f m, Heading=%.1f degrees",
                             state.poseMeters.getTranslation().getX(),
                             state.poseMeters.getTranslation().getY(),
                             state.poseMeters.getRotation().getDegrees());
    }

    /** @param trajectory Trajectory;
     *  @return Info about end point
     */
    public static String getEndInfo(final Trajectory trajectory)
    {
      return "End: " + getInfo(getEnd(trajectory));
    }

    /** Re-locate and rotate a trajectory
     *  @param previous Previous trajectory
     *  @param trajectory Trajectory to translates
     *  @return Transformed trajectory where initial state is at end of previous trajectory
     */
    public static Trajectory continueTrajectory(final Trajectory previous, final Trajectory trajectory)
    {
        return makeTrajectoryStartAt(trajectory, getEndPose(previous));
    }

    /** Re-locate and rotate a trajectory
     *  @param Original trajectory
     *  @param start Startpoint and heading
     *  @return Transformed trajectory where initial state is 'start'
     */
    public static Trajectory makeTrajectoryStartAt(final Trajectory trajectory, final Pose2d start)
    {
        // The relativeTo() methods in Pose2d and Trajectory are basically
        // a "substract" operation for X, Y and the rotation angle.
        // Assume a trajectory that moves from "5" to "6", and start is at "2".
        // The offset becomes "5" - "2" = "3"
        final Transform2d offset = start.minus(trajectory.getInitialPose());
        // Computing the "5 -> 6" trajectory relative to "3" becomes "2 -> 3".
        // So we get a trajectory that moves by "1" unit, starting at "2".
        return trajectory.transformBy(offset);
    }

    /** Revert a trajectory
     *
     *  Change end point to start point,
     *  next-to-last into second point.
     *  Inverts direction of velocities etc.
     *
     *  An original trajectory that moves 0 -> 1
     *  turns into one that moves 0 -> -1,
     *
     *  Meant to turn existing 'forward' trajectory
     *  into one that's used in 'reverse', going
     *  backwards.
     *
     * @param original Trajectory
     * @return Reversed trajectory
     */
    public static Trajectory reverse(final Trajectory original)
    {
        final List<State> orig_states = original.getStates();
        final List<State> reversed_states = new ArrayList<>();

        final State end = orig_states.get(orig_states.size()-1);
        final double end_x = end.poseMeters.getTranslation().getX();
        final double end_y = end.poseMeters.getTranslation().getY();
        final double duration = original.getTotalTimeSeconds();

        // Run through original states in reverse order
        for (int i=orig_states.size()-1;  i>=0;  --i)
        {
            final State orig = orig_states.get(i);
            reversed_states.add(new State(duration - orig.timeSeconds,
                                          -orig.velocityMetersPerSecond,
                                          orig.accelerationMetersPerSecondSq,
                                          new Pose2d(new Translation2d(orig.poseMeters.getTranslation().getX() - end_x,
                                                                       orig.poseMeters.getTranslation().getY() - end_y),
                                                     orig.poseMeters.getRotation()),
                                          orig.curvatureRadPerMeter));
        }
        return new Trajectory(reversed_states);
    }
}
