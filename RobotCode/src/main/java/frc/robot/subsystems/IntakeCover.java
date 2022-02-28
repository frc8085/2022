// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.IntakeCoverConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCover extends SubsystemBase {
    private final DoubleSolenoid m_intakeCoverSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            IntakeCoverConstants.kIntakeCoverSolenoidPorts[0],
            IntakeCoverConstants.kIntakeCoverSolenoidPorts[1]);

    /**  */
    private boolean isIntakeCoverDown = true;

    /** Grabs the intake cover. */
    public void openIntake() {
        m_intakeCoverSolenoid.set(kForward);
        isIntakeCoverDown = true;
    }

    /** Releases the intake cover. */
    public void closeIntake() {
        m_intakeCoverSolenoid.set(kReverse);
        isIntakeCoverDown = false;
    }
}