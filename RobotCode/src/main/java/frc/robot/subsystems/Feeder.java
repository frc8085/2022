// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Feeder extends SubsystemBase {
    private final CANSparkMax m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort, MotorType.kBrushless);

    /** The feeder subsystem for the robot. */
    public Feeder() {
    }

    public void runFeeder() {
        m_feederMotor.set(ShooterConstants.kFeederSpeed);
    }

    public void stopFeeder() {
        m_feederMotor.set(0);
    }

}
