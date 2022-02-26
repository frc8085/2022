// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private NetworkTableEntry readyToShoot;

  private final CANSparkMax m_shooterMotor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double kSetPoint, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** The shooter subsystem for the robot. */
  public Shooter() {
    m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort,
        MotorType.kBrushless);
    m_shooterMotor.restoreFactoryDefaults();
    m_encoder = m_shooterMotor.getEncoder();
    m_pidController = m_shooterMotor.getPIDController();

    // PID coefficients
    // kP = 6e-5;
    kP = 0.0001;
    kI = 0;
    kD = 0.001;
    kIz = 0;
    kFF = 0.0001761804087;
    kMaxOutput = 0;
    kMinOutput = -1;
    maxRPM = 5700;
    kSetPoint = ShooterConstants.kShooterTargetRPM[0];

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // board.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    readyToShoot = Shuffleboard.getTab("Shooter")
        .add("Ready to shoot", atSetpoint())
        .withPosition(2, 0)
        .withSize(2, 1)
        .getEntry();

  }

  @Override
  public void periodic() {

    // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    // if ((p != kP)) {
    // m_pidController.setP(p);
    // kP = p;
    // }
    // if ((i != kI)) {
    // m_pidController.setI(i);
    // kI = i;
    // }
    // if ((d != kD)) {
    // m_pidController.setD(d);
    // kD = d;
    // }
    // if ((iz != kIz)) {
    // m_pidController.setIZone(iz);
    // kIz = iz;
    // }
    // if ((ff != kFF)) {
    // m_pidController.setFF(ff);
    // kFF = ff;
    // }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {
    // m_pidController.setOutputRange(min, max);
    // kMinOutput = min;
    // kMaxOutput = max;
    // }

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
    SmartDashboard.putNumber("Setpoint", kSetPoint);
    readyToShoot.setBoolean(atSetpoint());
  }

  public void setSetpoint(double setPoint) {
    kSetPoint = setPoint;
    m_pidController.setReference(kSetPoint, CANSparkMax.ControlType.kVelocity);
  }

  public boolean atSetpoint() {
    double encoderValue = m_encoder.getVelocity();
    double tolerance = Math.abs(ShooterConstants.kShooterToleranceRPMPercent * kSetPoint);
    double setpoint = kSetPoint;
    double minLimit = setpoint - tolerance;
    double maxLimit = setpoint + tolerance;

    boolean withinLimits =
        // Don't consider us at setpoint for the 'motor off' case
        setpoint != 0 &&
        // Otherwise check if we're within limits
            encoderValue >= minLimit
            && encoderValue <= maxLimit;

    return withinLimits;
  }

  public void stopShooter() {
    m_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_shooterMotor.set(0);
  }
}
