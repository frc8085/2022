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
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort,
      MotorType.kBrushless);

  // Conveyor motor
  private final CANSparkMax m_ConveyorMotor = new CANSparkMax(IntakeConstants.kConveyorMotorPort,
      MotorType.kBrushless);

  // Run the intake
  public void runIntake() {
    m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
    m_ConveyorMotor.set(IntakeConstants.kConveyorSpeed);
  }

  // Stop the intake the intake
  public void stopIntake() {
    m_IntakeMotor.set(0);
    m_ConveyorMotor.set(0);
  }

  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
