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
    // public static final int kConveyorMotorPort1 = 8;
    // public static final int kConveyorMotorPort2 = 9;

    public static final double kIntakeSpeed = -0.75;
    public static final double kConveyorSpeed = 0.5;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPort = 7;
    public static final int kFeederMotorPort = 6;

    // Low shot
    // public static final double kShooterTargetRPM = -2300;


    // High Shot
    public static final double kShooterHighTargetRPM = 3200;

    public static final double kShooterHighToleranceRPM = Math.abs(0.05 * kShooterHighTargetRPM);

    public static final double kFeederSpeed = -0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

}
