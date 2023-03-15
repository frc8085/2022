// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 4;
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 3;

    // TODO. Decide ramp rate (seconds) for drive motors
    // {kRampRate} = Time in seconds to go from 0 to full throttle.
    public static final double kRampRate = 1;

    // Limits the rate of change of the signal (Joystick input) to
    // {kSlewRateLimit} units per second
    public static final double kSpeedSlewRateLimit = 10;
    public static final double kRotationSlewRateLimit = 10;
    public static final int kGyroChannel = 1;

    public static final double kWheelDiameterInches = 6;

    public static final double kWheelDiameterMeters = 0.1524;

    // The inches one wheel revolution travels
    public static final double kInchesPerWheelRevolution = kWheelDiameterInches * Math.PI;
    public static final double kMetersPerWheelRevolution = kWheelDiameterMeters * Math.PI;

    // We determined the motor revolutions per wheel revolution
    // by turning the wheel once and recording the encoder units measured
    public static final double kGearRatio = 10.75;

    // Inches per motor revolution
    public static final double kInchesPerMotorRevolution = kInchesPerWheelRevolution / kGearRatio;
    public static final double kMetersPerMotorRevolution = kMetersPerWheelRevolution / kGearRatio;

    // public static final double kGearRatio = 10.75;
    // public static final double kEncoderCPR = 42 * kGearRatio;
    // public static final double kEncoderDistancePerPulse =
    // (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    public static final double kAutoPositionTolerance = 3;
    public static final double kAutoGyroTolerance = 5;

    public static final double kReverse = -1;

    public static final double kRPMtoMPSFactor = 0.007979645340118075;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these values for your robot.

    public static final double ksVolts = 0.24149;

    // INCHES
    public static final double kvVoltSecondsPerInch = 0.074863;
    public static final double kaVoltSecondsSquaredPerInch = 0.020198;

    public static final double kTrackWidthInches = 21.75; // Known
    public static final double kTurnFactor = kTrackWidthInches * Math.PI / 360; // Known

    public static final double kPDriveVelInches = 0.10749;

    // METERS
    public static final double kvVoltSecondsPerMeter = 0.074863;
    public static final double kaVoltSecondsSquaredPerMeter = 0.020198;
    public static final double kPDriveVel = 8.5;

    public static final double kTrackWidthMeters = 0.55245; // Known
    public static final double kTurnFactorMeters = kTrackWidthMeters * Math.PI / 360; // Known

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 5;
    public static final int kConveyorMotorPort1 = 8;
    public static final int kConveyorMotorPort2 = 9;

    public static final double kIntakeSpeed = -.75;
    public static final double kConveyorSpeed = 0.5;

    // TODO. Decide peak current in AMPs for the intake motor
    // public static final double kPeakCurrent = 10;
    // TODO. Decide ramp rate (seconds) for intake motor
    // rate = Time in seconds to go from 0 to full throttle.
    public static final double kRampRate = 0.25;

    // Duration in seconds to keep running the load
    // cargo routine after release
    public static double kLoadLagSecs = 1;

  }

  public static final class IntakeCoverConstants {
    public static final int[] kIntakeCoverSolenoidPorts = new int[] { 6, 7 };

  }

  public static final class ClimberBrakeConstants {
    public static final int[] kClimberBrakeSolenoidPorts = new int[] { 4, 5 };

  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPort = 7;
    public static final int kFeederMotorPort = 6;

    public static final double kShooterToleranceRPMPercent = 0.10;
    public static final double kFeederSpeed = -0.5;

    // Shooter off speed
    public static final int kShooterOffSpeed = 0;

    // High shot speeds = 4200 can shoot into basketball net
    public static final int kFarSpeed = -4200;
    public static final int kTBDSpeed = -2500;
    public static final int kNearSpeed = -1900;

    // Low shot speeds
    // public static final int kBumpedFarSpeed = -4000;
    public static final int kBumpedFarSpeed = -3500;
    public static final int kBumpedTBDSpeed = -2500;
    public static final int kBumpedNearSpeed = -1900;

    // High shot delays
    public static final double kShooterOffDelay = 0;
    public static final double kFarDelay = 0.85;
    public static final double kTBDDelay = 0.85;
    public static final double kNearDelay = 0.85;

    // Low shot delays
    public static final double kBumpedFarDelay = 1;
    public static final double kBumpedTBDDelay = 1;
    public static final double kBumpedNearDelay = 1;

    public static final int[] kShooterTargetRPM = new int[] {
        kShooterOffSpeed, // 0, kShooterOff
        kNearSpeed, // 1, kTargetNear
        kFarSpeed, // 2, kTargetFar
        kTBDSpeed, // 3, kTargetTBD
        kBumpedNearSpeed, // 4, kTargetBumpedNear
        kBumpedFarSpeed, // 6, kTargetBumpedFar
        kBumpedTBDSpeed// 6, kTargetBumpedTBD
    };

    public static final double[] kShooterTargetDelay = new double[] {
        kShooterOffDelay, // 0, kShooterOff
        kNearDelay, // 1, kTargetNear
        kFarDelay, // 2, kTargetFar
        kTBDDelay, // 3, kTargetTBD
        kBumpedNearDelay, // 4, kTargetBumpedNear
        kBumpedFarDelay, // 6, kTargetBumpedFar
        kBumpedTBDDelay// 6, kTargetBumpedTBD
    };

    // Duration in seconds that the shooter should shoot
    public static final double kShootBurstTime = .5;
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorPort = 14;
  }

  public static final class AutoConstants {
    public static final boolean kPickupCargo = true;
    public static final boolean kDontPickupCargo = false;
    public static final int kStandStill = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /** Shooter off index in { @see ShooterConstants.kShooterTargetRPM } */
    public static final int kShooterOff = 0;

    /** High shot indices in { @see ShooterConstants.kShooterTargetRPM } */
    public static final int kTargetNear = 1;
    public static final int kTargetFar = 2;
    public static final int kTargetTBD = 3;

    /** Low shot indices in { @see ShooterConstants.kShooterTargetRPM } */
    public static final int kTargetBumpedNear = 4;
    public static final int kTargetBumpedFar = 5;
    public static final int kTargetBumpedTBD = 6;
  }

}
