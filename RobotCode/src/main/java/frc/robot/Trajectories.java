// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.robot.Constants.DriveConstants.kDriveKinematics;
import static frc.robot.Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.DriveConstants.kMaxVoltage;
import static frc.robot.Constants.DriveConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.DriveConstants.ksVolts;
import static frc.robot.Constants.DriveConstants.kvVoltSecondsPerMeter;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The trajectories we pre-programmed for autonomous
 */
public final class Trajectories {
    /**
     * First, we set up our contraints and configurations
     * This makes sure we don't drive past our max speed
     * or accelerate too fast
     */

    // Create a voltage constraint so we don't accelerate too fast
    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    ksVolts,
                    kvVoltSecondsPerMeter,
                    kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            kMaxVoltage);

    // Create config for trajectory
    private static final TrajectoryConfig config = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    /**
     * MANUALLY DEFINED trajectories
     * Now we actually define our different trajectories.
     * Unlike the PathWeaver JSON output files, we enter these waypoints inline
     * We only do this because PathWeaver generated trajectories cannot have
     * contraints (we may drive or accellerate to fast)
     */

    /**
     * 🔵
     * We don't need a kPickupFirst because we start with one cargo loaded
     */

    /** 🔵🔵 */
    public static final Trajectory kPickupSecond = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(
                    Units.inchesToMeters(298.15752741774673),
                    Units.inchesToMeters(-245.994),
                    new Rotation2d(0)),
            // Pass through this interior waypoint
            List.of(new Translation2d(
                    Units.inchesToMeters(316.5509072781655),
                    Units.inchesToMeters(-277.59337986041874))),
            // End end at the second cargo pickup spot
            new Pose2d(
                    Units.inchesToMeters(302.873778664008),
                    Units.inchesToMeters(-308.24901296111665),
                    new Rotation2d(0)),
            // Pass config
            config);

    /** 🔵🔵🔵 */
    public static final Trajectory kPickupThird = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(
                    Units.inchesToMeters(302.873778664008),
                    Units.inchesToMeters(-308.24901296111665),
                    new Rotation2d(0)),
            // Pass through this interior waypoint
            List.of(
                    new Translation2d(
                            Units.inchesToMeters(272.6897706879362),
                            Units.inchesToMeters(-310.1355134596211)),
                    new Translation2d(
                            Units.inchesToMeters(234.95976071784648),
                            Units.inchesToMeters(-286.0826321036889))),
            // End end at the second cargo pickup spot
            new Pose2d(
                    Units.inchesToMeters(201.47437686939185),
                    Units.inchesToMeters(-252.59724825523432),
                    new Rotation2d(0)),
            // Pass config
            config);

    /** 🔵🔵🔵🔵 */
    public static final Trajectory kPickupFourth = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(
                    Units.inchesToMeters(201.47437686939185),
                    Units.inchesToMeters(-252.59724825523432),
                    new Rotation2d(0)),
            // Pass through this interior waypoint
            List.of(
                    new Translation2d(
                            Units.inchesToMeters(154.31186440677965),
                            Units.inchesToMeters(-244.10799601196413)),
                    new Translation2d(
                            Units.inchesToMeters(98.66009970089733),
                            Units.inchesToMeters(-251.65399800598206))),
            // End end at the second cargo pickup spot
            new Pose2d(
                    Units.inchesToMeters(58.572),
                    Units.inchesToMeters(-269.576),
                    new Rotation2d(0)),
            // Pass config
            config);

}
