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
        public enum TrajectoryType {
                PATHWEAVER, SAFETY
        }

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
         * MANUALLY DEFINED trajectories. Unlike the PathWeaver JSON output files, we
         * enter these waypoints inline. We only do this because PathWeaver generated
         * trajectories cannot have contraints (we may drive or accellerate to fast)
         */

        // 🔵 We don't need a kPickupFirst because we start with one cargo loaded

        /** 🔵 🔵 */
        public static final Trajectory kPickupSecond = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        Waypoints.kStartingPose,
                        List.of(), // No interior points for first move
                        // End at the second cargo pickup spot
                        Waypoints.kSecondCargo,
                        // Pass config
                        config);

        /** RETURN path from 2nd cargo */
        public static final Trajectory kReturnFromSecond = TrajectoryGenerator.generateTrajectory(
                        // Start at the second cargo
                        Waypoints.kSecondCargo,
                        List.of(), // No interior points for return
                        // End at the starting spot
                        Waypoints.kStartingPose,
                        // Pass config and set it reversed
                        config.setReversed(true));

        /** 🔵 🔵 🔵 */
        public static final Trajectory kPickupThird = TrajectoryGenerator.generateTrajectory(
                        // Start at the second cargo
                        Waypoints.kSecondCargo,
                        // Pass through these interior waypoints
                        List.of(new Translation2d(
                                        Units.inchesToMeters(235),
                                        Units.inchesToMeters(38))),
                        // End at the third cargo spot
                        Waypoints.kThirdCargo,
                        // Pass config
                        config);

        /** 🔵 🔵 🔵 🔵 */
        public static final Trajectory kPickupFourth = TrajectoryGenerator.generateTrajectory(
                        // Start at the third cargo
                        Waypoints.kThirdCargo,
                        // Pass through these interior waypoints
                        List.of(new Translation2d(
                                        Units.inchesToMeters(99),
                                        Units.inchesToMeters(72))),
                        // End at the fourth cargo pickup spot
                        Waypoints.kFourthCargo,
                        // Pass config
                        config);

        /** RETURN path from fourth cargo */
        public static final Trajectory kReturn = TrajectoryGenerator.generateTrajectory(
                        // Start at the fourth/fifth cargo
                        Waypoints.kFourthCargo,
                        // Pass through these interior waypoints
                        List.of(new Translation2d(
                                        Units.inchesToMeters(130),
                                        Units.inchesToMeters(63))),
                        // End at the starting position
                        Waypoints.kStartingPose,
                        // Pass config
                        config.setReversed(true));

}
