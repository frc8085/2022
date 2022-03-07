// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */

    // Intake motor
    private final CANSparkMax m_IntakeMotor = new CANSparkMax(kIntakeMotorPort, MotorType.kBrushless);

    public Intake() {
        m_IntakeMotor.setOpenLoopRampRate(kRampRate);
    }

    // Run the intake forward
    public void runIntake() {
        m_IntakeMotor.set(kIntakeSpeed);
    }

    // Run the intake reverse
    public void reverseIntake() {
        m_IntakeMotor.set(-kIntakeSpeed);
    }

    // Stop the intake the intake
    public void stopIntake() {
        m_IntakeMotor.set(0);
    }

}
