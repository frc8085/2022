// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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

    // The inches one wheel revolution travels
    public static final double kInchesPerWheelRevolution = kWheelDiameterInches * Math.PI;

    // We determined the motor revolutions per wheel revolution
    // by turning the wheel once and recording the encoder units measured
    public static final double kGearRatio = 10.75;

    // Inches per motor revolution
    public static final double kInchesPerMotorRevolution = kInchesPerWheelRevolution / kGearRatio;

    // public static final double kGearRatio = 10.75;
    // public static final double kEncoderCPR = 42 * kGearRatio;
    // public static final double kEncoderDistancePerPulse =
    // (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    public static final double kAutoPositionTolerance = 3;
    public static final double kTrackWidthInches = 21.75;
    public static final double kReverse = -1;

    public static final double kTurnFactor = kTrackWidthInches * Math.PI / 360;

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

    // High shot speeds
    public static final int kHighFarSpeed = -3800;
    public static final int kHighAngledSpeed = -3500;
    public static final int kHighNearSpeed = -2100;

    // Low shot speeds
    public static final int kLowFarSpeed = -4000;
    public static final int kLowAngledSpeed = -2000;
    public static final int kLowNearSpeed = -1800;

    public static final int[] kShooterTargetRPM = new int[] {
        kShooterOffSpeed, // 0, kShooterOff
        kHighNearSpeed, // 1, kTargetHighNear
        kHighFarSpeed, // 2, kTargetHighFar
        kHighAngledSpeed, // 3, kTargetHighAngled
        kLowNearSpeed, // 4, kTargetLowNear
        kLowFarSpeed, // 6, kTargetLowFar
        kLowAngledSpeed// 6, kTargetLowAngled
    };

    // Duration in seconds that the shooter should shoot
    public static final double kShootBurstTime = .5;
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorPort = 14;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /** Shooter off index in { @see ShooterConstants.kShooterTargetRPM } */
    public static final int kShooterOff = 0;

    /** High shot indices in { @see ShooterConstants.kShooterTargetRPM } */
    public static final int kTargetHighNear = 1;
    public static final int kTargetHighFar = 2;
    public static final int kTargetHighAngled = 3;

    /** Low shot indices in { @see ShooterConstants.kShooterTargetRPM } */
    public static final int kTargetLowNear = 4;
    public static final int kTargetLowFar = 5;
    public static final int kTargetLowAngled = 6;

  }

}
