// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.OIConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private boolean TUNING_MODE = false;
    private final CANSparkMax m_shooterMotor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private double kSetPoint, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    /**
     * Shooting Mode definition
     * shootMode is an integer corresponding to the different types of targets
     * shootingMode translates the integer into a string so we can display it
     * in the dashboard
     */

    private int shootMode = kShooterOff;
    private boolean bumpShoot = false;

    private static final Map<Integer, String> shootingMode = new HashMap<Integer, String>() {
        {
            put(kShooterOff, "Shooting mode not selected");

            put(kTargetNear, "Near");
            put(kTargetFar, "------------------------- Far");
            put(kTargetTBD, "---Mid---");

            put(kTargetBumpedNear, "High - Near");
            put(kTargetBumpedFar, "High -------------------- Far");
            put(kTargetBumpedTBD, "High ---Mid---");
        }
    };

    /** The shooter subsystem for the robot. */
    public Shooter() {
        m_shooterMotor = new CANSparkMax(kShooterMotorPort,
                MotorType.kBrushless);
        m_shooterMotor.restoreFactoryDefaults();
        m_encoder = m_shooterMotor.getEncoder();
        m_pidController = m_shooterMotor.getPIDController();

        // PID coefficients
        kP = 0.0004;
        kI = 0;
        kD = 0.004;
        kIz = 0;
        kFF = 0.0001761804087;
        kMaxOutput = 0;
        kMinOutput = -1;
        kSetPoint = kShooterTargetRPM[0];

        // Set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // Add relevant displays to the Operator dashboard
        configureOperatorDashboard();

        // If we're fine-tuning PID Constants, the display them on the dashboard
        if (TUNING_MODE) {
            addPIDToDashboard();
        }

    }

    private void configureOperatorDashboard() {
        // Add the selected shooting mode to the Operator dashboard
        SmartDashboard.putString("Shooting Mode", shootingMode.get(shootMode));

        // Add the setpoint but only if in TUNINGMODE
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Setpoint", kSetPoint);
        }

        // Add ready to shoot indicator to the Operator dashboard
        SmartDashboard.putBoolean("Ready to shoot", atSetpoint());
    }

    private void addPIDToDashboard() {
        // Display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

    }

    private void readPIDTuningFromDashboard() {

        // Read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
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
    }

    @Override
    public void periodic() {
        // If we're fine-tuning PID Constants, read and apply updates from the dashboard
        if (TUNING_MODE) {
            readPIDTuningFromDashboard();
            SmartDashboard.putNumber("Encoder velocity Shooter", m_encoder.getVelocity());

        }
    }

    public void setSetpointFromDistance(DoubleSupplier distanceSupplier) {
        // Empirically derived formula
        double distance = distanceSupplier.getAsDouble() + 15;
        double autoSetpoint = 0.1607 * Math.pow(distance, 2) - 28.274 * distance +
                4991.1;

        // When we're too close the setpoint formla is unreliable. Fix the speed
        // instead.
        double setpoint = distance < 100 ? 3550 : autoSetpoint;
        setSetpoint(-1 * setpoint);
    }

    public void setSetpoint(double setPoint) {
        kSetPoint = Math.max(setPoint, -4500);
        m_pidController.setReference(kSetPoint, CANSparkMax.ControlType.kVelocity);
    }

    public boolean atSetpoint() {
        double encoderValue = m_encoder.getVelocity();
        // double tolerance = Math.abs(kShooterToleranceRPMPercent * kSetPoint);
        double tolerance = 300;
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

    public void setShootingMode(int mode) {
        // First, update the shootMode to be the requested mode
        shootMode = mode;

        // Then decide whether to bump the speed
        if (bumpShoot) {
            setBumped();
        } else {
            setNormal();
        }
    }

    private void updateSetpoint() {
        // Update the setpoint to whatever the current shooting mode is
        setSetpoint(kShooterTargetRPM[shootMode]);
    }

    /**
     * Used when holding down the 'Bump' control
     * This will switch whatever shooting mode you're in to "Bumped"
     */
    public void setBumped() {
        bumpShoot = true;
        int mode = shootMode;

        switch (mode) {
            case kTargetNear:
                shootMode = kTargetBumpedNear;
                break;
            case kTargetFar:
                shootMode = kTargetBumpedFar;
                break;
            case kTargetTBD:
                shootMode = kTargetBumpedTBD;
                break;
        }
        updateSetpoint();
    }

    /**
     * Used when NOT holding down the 'Bump' control
     * This will switch whatever shooting mode you're in to it's normal setpoint
     */
    public void setNormal() {
        bumpShoot = false;
        int mode = shootMode;

        switch (mode) {
            case kTargetBumpedNear:
                shootMode = kTargetNear;
                break;
            case kTargetBumpedFar:
                shootMode = kTargetFar;
                break;
            case kTargetBumpedTBD:
                shootMode = kTargetTBD;
                break;
        }
        updateSetpoint();
    }

}
