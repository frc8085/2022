// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private final CANSparkMax m_climberMotor = new CANSparkMax(ClimberConstants.kClimberMotorPort, MotorType.kBrushless);

  /** Creates a new Climber. */
  public Climber() {
  }

  // Start the climber motor
  public void climb() {
    m_climberMotor.set(ClimberConstants.kClimberMotorSpeed);
  }

  // Stop the climber motor
  public void stopClimb() {
    m_climberMotor.set(0);
  }

}
