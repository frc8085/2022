// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.ClimberBrake;

/** An example command that uses an example subsystem. */
public class Climb extends CommandBase {
  private final Climber m_climber;
  private final IntakeCover m_intakeCover;
  private final ClimberBrake m_climberBrake;

  public Climb(Climber climber, IntakeCover intakeCover, ClimberBrake climberBrake) {
    m_climber = climber;
    m_intakeCover = intakeCover;
    m_climberBrake = climberBrake;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeCover.openIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.climb();
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
