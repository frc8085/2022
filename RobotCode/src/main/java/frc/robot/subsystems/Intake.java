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
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushed);
  // Conveyor motors
  // private final MotorControllerGroup m_ConveyorMotors = new
  // MotorControllerGroup(
  // new CANSparkMax(IntakeConstants.kConveyorMotorPort1,
  // MotorType.kBrushless),
  // new CANSparkMax(IntakeConstants.kConveyorMotorPort2,
  // MotorType.kBrushless));

  // Run the intake
  public void runIntake() {
    System.out.println("<RunIntake> at speed ***" + IntakeConstants.kIntakeSpeed + "******");
    m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
    // m_ConveyorMotors.set(IntakeConstants.kConveyorSpeed);
  }

  // Stop the intake the intake
  public void stopIntake() {
    System.out.println("</StopIntake> ******");
    m_IntakeMotor.set(0);
    // m_ConveyorMotors.set(0);
  }

  public Intake() {

  }

}
