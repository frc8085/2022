// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.OIConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
     * in the Shuffleboard entry shotingModeDisplay
     */
    private NetworkTableEntry shootingModeDisplay;
    private NetworkTableEntry setpointDisplay;
    private NetworkTableEntry readyToShoot;

    private int shootMode = kShooterOff;
    private boolean bumpShoot = false;

    private static final Map<Integer, String> shootingMode = new HashMap<Integer, String>() {
        {
            put(kShooterOff, "Shooting mode not selected");

            put(kTargetNear, "▔ Near");
            put(kTargetFar, "▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔ Far");
            put(kTargetTBD, "▔▔/tbd/▔▔");

            put(kTargetBumpedNear, "↑bump _ Near");
            put(kTargetBumpedFar, "↑bump ____________________ Far");
            put(kTargetBumpedTBD, "↑bump __/tbd/__");
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
        kP = 0.0001;
        kI = 0;
        kD = 0.001;
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

        // Add relevant displays to the teleop dashboard
        configureTeleopDashboard();

        // If we're fine-tuning PID Constants, the display them on the dashboard
        if (TUNING_MODE) {
            addPIDToDashboard();
        }

    }

    private void configureTeleopDashboard() {
        // Add the selected shooting mode to the Teleop dashboard
        shootingModeDisplay = Shuffleboard.getTab("Teleop")
                .add("Shooting Mode", shootingMode.get(shootMode))
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();

        // Add the setpoint
        setpointDisplay = Shuffleboard.getTab("Teleop")
                .add("Setpoint", kSetPoint)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();

        // Add ready to shoot indicator to the Teleop dashboard
        readyToShoot = Shuffleboard.getTab("Teleop")
                .add("Ready to shoot", atSetpoint())
                .withPosition(2, 0)
                .withSize(4, 2)
                .getEntry();
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
        // Display the current shooting mode and set point
        shootingModeDisplay.setString(shootingMode.get(shootMode));
        setpointDisplay.setNumber(kSetPoint);

        // Turn the display GREEN on the 'Shooter' tab if we're ready to shoot
        readyToShoot.setBoolean(atSetpoint());

        // If we're fine-tuning PID Constants, read and apply updates from the dashboard
        if (TUNING_MODE) {
            readPIDTuningFromDashboard();
        }
    }

    public void setSetpoint(double setPoint) {
        kSetPoint = setPoint;
        m_pidController.setReference(kSetPoint, CANSparkMax.ControlType.kVelocity);
    }

    public boolean atSetpoint() {
        double encoderValue = m_encoder.getVelocity();
        double tolerance = Math.abs(kShooterToleranceRPMPercent * kSetPoint);
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
