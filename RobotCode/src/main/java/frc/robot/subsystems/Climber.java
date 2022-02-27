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
  private XboxController m_OperatorController;

  private final CANSparkMax m_climberMotor = new CANSparkMax(ClimberConstants.kClimberMotorPort, MotorType.kBrushless);

  /** Creates a new Climber. */
  public Climber(XboxController operatorController) {
    m_OperatorController = operatorController;
  }

  /*
   * Will check if the lock trigger is pressed
   * 
   * @return whether the unlock trigger is being pressed
   * Unlock trigger is the Left trigger
   */

  public boolean isUnlocked(double leftTrigger) {
    return leftTrigger > 0;
  }

  /*
   * Will run the climber as long as it's unlocked
   */
  public void climb() {
    double leftTrigger = m_OperatorController.getLeftTriggerAxis();
    double leftY = m_OperatorController.getLeftY();

    // Only climb if unlocked
    if (isUnlocked(leftTrigger)) {
      m_climberMotor.set(leftY);
    } else {
      stopClimb();
    }
  }

  // Stop the climber motor
  public void stopClimb() {
    m_climberMotor.set(0);
  }

}
