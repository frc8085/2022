// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GTADrive;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class DriveStraight extends PIDCommand {
  private final GTADrive m_drivetrain;

  static double kP = 0.01;
  static double kI = 0;
  static double kD = 0;

  /**
   * Create a new DriveStraight command.
   *
   * @param distance The distance to drive (inches)
   */
  public DriveStraight(double distance, GTADrive drivetrain) {
    super(
        new PIDController(kP, kI, kD),
        drivetrain::getDistance,
        distance, d -> drivetrain.drive(d, d));

    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

    getController().setTolerance(DriveConstants.kAutoPositionTolerance);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Left Distance", m_drivetrain.getLeftEncoderDistanceInches());
    SmartDashboard.putNumber("Right Distance", m_drivetrain.getRightEncoderDistanceInches());
    SmartDashboard.putNumber("Distance traveled", m_drivetrain.getDistance());
    super.execute();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // Get everything in a safe starting state.
    m_drivetrain.reset();
    super.initialize();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}