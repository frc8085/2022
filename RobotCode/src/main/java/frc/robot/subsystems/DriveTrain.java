// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
        /** Creates a new DriveTrain. */

        // Left motors
        private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
                        new CANSparkMax(DriveConstants.kLeftMotor1Port,
                                        MotorType.kBrushless),
                        new CANSparkMax(DriveConstants.kLeftMotor2Port,
                                        MotorType.kBrushless));

        // Right motors
        private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
                        new CANSparkMax(DriveConstants.kRightMotor1Port,
                                        MotorType.kBrushless),
                        new CANSparkMax(DriveConstants.kRightMotor2Port,
                                        MotorType.kBrushless));

        // The robot's drive
        private final DifferentialDrive m_drive = new DifferentialDrive(
                        m_leftMotors, m_rightMotors);

        // Drive the robot using ArcadeDrive command
        // TODO:
        // https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax#setOpenLoopRampRate(double
        public void ArcadeDrive(double speed, double rotation) {
                m_drive.arcadeDrive(-1 * (Math.pow(speed, 3)), -rotation);
        }

        public DriveTrain() {
                m_leftMotors.setInverted(true);
        }
}
