// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private XboxController m_operatorController;
    DigitalInput limitSwitch = new DigitalInput(0);

    // Climber is locked by default
    private boolean isLocked = true;

    private final CANSparkMax m_climberMotor = new CANSparkMax(kClimberMotorPort, MotorType.kBrushless);

    /** Creates a new Climber. */
    public Climber(XboxController operatorController) {
        m_operatorController = operatorController;
    }

    /* Lock the climber */
    public void lockClimber() {
        isLocked = true;
    }

    /* Unlock the climber */
    public void unlockClimber() {
        isLocked = false;
    }

    public boolean isLocked() {
        return isLocked;
    }

    /*
     * Will run the climber as long as it's unlocked
     * and we haven't hit the limit switch
     */
    public void climb() {
        double rightY = m_operatorController.getRightY();

        // Reasons for blocking climber:
        // 1. Either it is locked
        // 2. Or the top limit switch is tripped
        // 3. Or the bottom limit switch is tripped
        boolean blockClimb = isLocked;
        // || toplimitSwitch.get() || bottomlimitSwitch.get();

        // Only climb if not blocked
        if (isLocked) {
            stopClimb();
        } else {
            // Deadband to remove drift when climbing
            if (Math.abs(rightY) < 0.1) {
                stopClimb();
            } else {
                if (limitSwitch.get()) {
                    if ((rightY) > 0) {
                        stopClimb();
                    } else {
                        m_climberMotor.set(rightY * 0.8);
                    }
                } else {
                    m_climberMotor.set(rightY * 0.8);
                }
            }
        }

    }

    // Stop the climber motor
    public void stopClimb() {
        m_climberMotor.set(0);
    }

}
