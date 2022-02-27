// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  // Intake motor
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);

  // Run the intake forward
  public void runIntake() {
    m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
  }

  // Run the intake reverse
  public void reverseIntake() {
    m_IntakeMotor.set(-IntakeConstants.kIntakeSpeed);
  }

  // Stop the intake the intake
  public void stopIntake() {
    m_IntakeMotor.set(0);
  }

  public Intake() {
    // m_IntakeMotor.setSecondaryCurrentLimit(IntakeConstants.kPeakCurrent);
    m_IntakeMotor.setOpenLoopRampRate(IntakeConstants.kRampRate);
  }

}
