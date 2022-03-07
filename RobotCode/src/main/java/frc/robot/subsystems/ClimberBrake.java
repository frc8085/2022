// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.Constants.ClimberBrakeConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberBrake extends SubsystemBase {
    private final DoubleSolenoid m_climberBrakeSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            kClimberBrakeSolenoidPorts[0],
            kClimberBrakeSolenoidPorts[1]);

    /** Grabs the intake cover. */
    public void lockClimber() {
        m_climberBrakeSolenoid.set(kForward);
    }

    /** Releases the intake cover. */
    public void unlockClimber() {
        m_climberBrakeSolenoid.set(kReverse);
    }
}