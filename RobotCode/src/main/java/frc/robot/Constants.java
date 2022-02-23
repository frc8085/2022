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
    // rate = Time in seconds to go from 0 to full throttle.
    public static final double kRampRate = 0.5;

    public static final int[] kRightEncoderPorts = new int[] { 2, 4 };
    public static final int[] kLeftEncoderPorts = new int[] { 1, 3 };

    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 5;
    public static final int kConveyorMotorPort1 = 8;
    public static final int kConveyorMotorPort2 = 9;

    public static final double kIntakeSpeed = -.75;
    public static final double kConveyorSpeed = 0.75;

    // TODO. Decide peak current in AMPs for the intake motor
    public static final double kPeakCurrent = 10;
    // TODO. Decide ramp rate (seconds) for intake motor
    // rate = Time in seconds to go from 0 to full throttle.
    public static final double kRampRate = 0.25;

  }

  public static final class HatchConstants {
    public static final int[] kHatchSolenoidPorts = new int[] { 6, 7 };

  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPort = 7;
    public static final int kFeederMotorPort = 6;

    public static final double kShooterToleranceRPMPercent = 0.10;
    public static final double kFeederSpeed = -0.5;
    public static final int[] kShooterTargetRPM = new int[] { 0, -3200, -3800, -2300 };
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorPort = 14;
    public static double kClimberMotorSpeed = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kTargetHighNear = 1;
    public static final int kTargetHighFar = 2;
    public static final int kTargetLow = 3;
    public static final int kShooterOff = 0;
  }

}
