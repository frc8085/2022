// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  // Left motors
  private final CANSparkMax left1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax left2 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);

  // Right motors
  private final CANSparkMax right1 = new CANSparkMax(DriveConstants.kRightMotor1Port,
      MotorType.kBrushless);
  private final CANSparkMax right2 = new CANSparkMax(DriveConstants.kRightMotor2Port,
      MotorType.kBrushless);

  // Create motor groups
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(left1, left2);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(right1, right2);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(
      m_leftMotors, m_rightMotors);

  // Drive the robot using ArcadeDrive command
  public void ArcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(-1 * (Math.pow(speed, 3)), -rotation);
  }

  public DriveTrain() {
    m_leftMotors.setInverted(true);
    left1.setOpenLoopRampRate(DriveConstants.kRampRate);
    left2.setOpenLoopRampRate(DriveConstants.kRampRate);
    right1.setOpenLoopRampRate(DriveConstants.kRampRate);
    right2.setOpenLoopRampRate(DriveConstants.kRampRate);
  }
}
