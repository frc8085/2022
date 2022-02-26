// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private XboxController m_peratorController;
  // Climber is locked by default
  private boolean isUnlocked = false;

  private final CANSparkMax m_climberMotor = new CANSparkMax(ClimberConstants.kClimberMotorPort, MotorType.kBrushless);

  /** Creates a new Climber. */
  public Climber(XboxController operatorController) {
    m_peratorController = operatorController;
  }

  /*
   * Unlocks the climber
   */
  public void unlockClimber() {
    isUnlocked = true;
  }

  /*
   * Will run the climber as long as it's unlocked
   */
  public void climb() {
    double rightY = m_peratorController.getRightY();
    // Only climb if unlocked
    if (isUnlocked) {
      m_climberMotor.set(rightY);
    } else {
      stopClimb();
    }
  }

  // Stop the climber motor
  public void stopClimb() {
    m_climberMotor.set(0);
  }

}
