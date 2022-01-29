// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private final PWMSparkMax motorLeft1 = new PWMSparkMax(Constants.MOTOR_LEFT_1_ID);
  private final PWMSparkMax motorLeft2 = new PWMSparkMax(Constants.MOTOR_LEFT_2_ID);
  private final PWMSparkMax motorRight1 = new PWMSparkMax(Constants.MOTOR_RIGHT_1_ID);
  private final PWMSparkMax motorRight2 = new PWMSparkMax(Constants.MOTOR_RIGHT_2_ID);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(motorLeft1, motorLeft2);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(motorRight1, motorRight2);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(
      leftMotors, rightMotors);

  public void ArcadeDrive(double speed, double rotation) {
    m_robotDrive.arcadeDrive(speed, rotation);
  }

  public DriveTrain() {
    motorRight2.setInverted(true);
    motorLeft2.setInverted(true);
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
