// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArcadeDriver extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_left_Y;
  private final DoubleSupplier m_right_X;


  /**
   * Creates a new ArcadeDriver.
   *
   * @param subsystem The subsystem used by this command.
   */

public ArcadeDriver(DriveTrain subsystem, DoubleSupplier left_Y, DoubleSupplier right_X) {
    m_driveTrain = subsystem;
    m_left_Y = left_Y;
    m_right_X = right_X;
    // Use addRequiremets() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.ArcadeDrive(m_left_Y.getAsDouble(), m_right_X.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
