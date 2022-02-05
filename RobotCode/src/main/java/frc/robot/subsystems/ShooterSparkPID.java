// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSparkPID extends SubsystemBase {
  private final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController = m_shooterMotor.getPIDController();
  private RelativeEncoder m_encoder = m_shooterMotor.getEncoder();
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, kSetpoint;

  /** The shooter subsystem for the robot. */
  public ShooterSparkPID() {
    // PID coefficients
    // kP = 6e-5;
    kP = 1;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = -5700;
    kSetpoint = -5250;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("SETPOINT", kSetpoint);

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
    double setpoint = SmartDashboard.getNumber("SETPOINT", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((setpoint != kSetpoint)) {
      m_pidController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
      kSetpoint = setpoint;
    }
    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      kIz = iz;
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
     * com.revrobotics.CANSparkMax.ControlType.kVelocity
     * com.revrobotics.CANSparkMax.ControlType.kVoltage
     */

    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }

  public void setSetpoint() {
    m_pidController.setReference(kSetpoint, CANSparkMax.ControlType.kVelocity);
  }

  // public boolean atSetpoint() {
  // return false
  // }

  public void stopShooter() {
    kSetpoint = 0;
    m_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  // public boolean atSetpoint() {
  // }

  public void runFeeder() {
    m_feederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
  }

}
