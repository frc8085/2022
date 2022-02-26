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
  private boolean isLocked = true;

  private final CANSparkMax m_climberMotor = new CANSparkMax(ClimberConstants.kClimberMotorPort, MotorType.kBrushless);

  /** Creates a new Climber. */
  public Climber(XboxController operatorController) {
    m_peratorController = operatorController;
  }

  /* Lock the climber */
  public void lockClimber() {
    isLocked = true;
  }

  /* Unlock the climber */
  public void unlockClimber() {
    isLocked = false;
  }

  /*
   * Will run the climber as long as it's unlocked
   */
  public void climb() {
    double rightY = m_peratorController.getRightY();

    // Only climb if unlocked
    if (isLocked) {
      stopClimb();
    } else {
      m_climberMotor.set(rightY * 0.3);
    }
  }

  // Stop the climber motor
  public void stopClimb() {
    m_climberMotor.set(0);
  }

}
