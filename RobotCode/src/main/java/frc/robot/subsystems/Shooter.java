// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_shooterMotor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double kPHigh, kIHigh, kDHigh, kIzHigh, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** The shooter subsystem for the robot. */
  public Shooter() {
    m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort,
        MotorType.kBrushless);
    m_shooterMotor.restoreFactoryDefaults();
    m_encoder = m_shooterMotor.getEncoder();
    m_pidController = m_shooterMotor.getPIDController();

    // PID coefficients
    // kP = 6e-5;
    kPHigh = 0.0001;
    kIHigh = 0;
    kDHigh = 0.001;
    kIzHigh = 0;
    kFF = 0.0001761804087;
    kMaxOutput = 1;
    kMinOutput = 0;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kPHigh);
    m_pidController.setI(kIHigh);
    m_pidController.setD(kDHigh);
    m_pidController.setIZone(kIzHigh);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kPHigh);
    SmartDashboard.putNumber("I Gain", kIHigh);
    SmartDashboard.putNumber("D Gain", kDHigh);
    SmartDashboard.putNumber("I Zone", kIzHigh);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

  }

  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kPHigh)) {
      m_pidController.setP(p);
      kPHigh = p;
    }
    if ((i != kIHigh)) {
      m_pidController.setI(i);
      kIHigh = i;
    }
    if ((d != kDHigh)) {
      m_pidController.setD(d);
      kDHigh = d;
    }
    if ((iz != kIzHigh)) {
      m_pidController.setIZone(iz);
      kIzHigh = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four
     * parameters:
     * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     * com.revrobotics.CANSparkMax.ControlType.kPosition
     * com.revrobotics.CANSparkMax.ControlType.kVoltage
     * com.revrobotics.CANSparkMax.ControlType.kVelocity
     */

    SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
  }

  public void setSetpoint() {
    m_pidController.setReference(ShooterConstants.kShooterHighTargetRPM, CANSparkMax.ControlType.kVelocity);
  }

  public boolean atSetpoint() {
    double encoderValue = m_encoder.getVelocity();
    double tolerance = ShooterConstants.kShooterHighToleranceRPM;
    double setpoint = ShooterConstants.kShooterHighTargetRPM;
    double minLimit = setpoint - tolerance;
    double maxLimit = setpoint + tolerance;
    boolean withinLimits = encoderValue >= minLimit && encoderValue <= maxLimit;

    return withinLimits;
  }

  public void stopShooter() {
    m_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
}
