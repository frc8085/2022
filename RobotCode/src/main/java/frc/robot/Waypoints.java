// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Waypoints around the field that we use for trajectories
 */
public final class Waypoints {
    /** Possible starting points */
    public enum StartingPosition {
        UP_AGAINST_HUB, PARALLEL_TO_WALL
    }

    /** CHOOSE STARTING POSITION HERE ************ */
    public static StartingPosition kStartingPosition = StartingPosition.PARALLEL_TO_WALL;
    /****************************************** */

    public static double kStartingRotation2d = getStartingRotation2d();

    /** 🔵 Starting pose with first cargo loaded */
    public static final Pose2d kStartingPose = getStartingPose();

    /** 🔵 🔵 */
    public static final Pose2d kSecondCargo = new Pose2d(
            Units.inchesToMeters(312),
            Units.inchesToMeters(20),
            // Clockwize = Negative; Counterclockwize = Positive
            // Update the rotation relative to our starting rotation
            new Rotation2d(Units.degreesToRadians(0) - kStartingRotation2d));

    /** 🔵 🔵 🔵 */
    public static final Pose2d kThirdCargo = new Pose2d(
            Units.inchesToMeters(213),
            Units.inchesToMeters(74),
            new Rotation2d(Units.degreesToRadians(-90) - kStartingRotation2d));

    /** 🔵 🔵 🔵 🔵 */
    public static final Pose2d kFourthCargo = new Pose2d(
            Units.inchesToMeters(50),
            Units.inchesToMeters(54),
            new Rotation2d(Units.degreesToRadians(-46) - kStartingRotation2d));

    private static Pose2d getStartingPose() {
        Pose2d startingPose;
        switch (kStartingPosition) {
            case PARALLEL_TO_WALL:
                startingPose = new Pose2d(
                        Units.inchesToMeters(312),
                        Units.inchesToMeters(63),
                        new Rotation2d(kStartingRotation2d));
                break;
            case UP_AGAINST_HUB:
                startingPose = new Pose2d(
                        Units.inchesToMeters(296),
                        Units.inchesToMeters(82),
                        new Rotation2d(kStartingRotation2d));
                break;
            default:
                startingPose = new Pose2d(
                        Units.inchesToMeters(312),
                        Units.inchesToMeters(63),
                        new Rotation2d(kStartingRotation2d));
        }

        return startingPose;

    }

    private static double getStartingRotation2d() {
        /**
         * We'll consider 0° to be the position where our robot's intake is
         * parallel to the wall of the field
         */

        /**
         * When we start the match, we're angled at
         * --- 0° if we're parallel to the wall / 2nd cargo
         * --- 20° if we're parallel to the hub
         */

        // Clockwize = Negative; Counterclockwize = Positive
        double startingRotation2d;
        switch (kStartingPosition) {
            case PARALLEL_TO_WALL:
                startingRotation2d = Units.degreesToRadians(0);
                break;
            case UP_AGAINST_HUB:
                startingRotation2d = Units.degreesToRadians(-20);
                break;
            default:
                startingRotation2d = 0;
        }
        return startingRotation2d;
    }

}
