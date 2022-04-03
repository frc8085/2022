// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.DriveConstants.*;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Aim using the limelight degree rotation to target
 */
public class AutoAimWithLimelight extends CommandBase {
    private final GTADrive m_drive;
    private final Limelight m_limelight;

    public AutoAimWithLimelight(GTADrive drive, Limelight limelight) {
        // Require the drive and limelight
        m_drive = drive;
        m_limelight = limelight;
        addRequirements(m_drive, m_limelight);
    }

    @Override
    public void execute() {
        super.execute();
        double turnToDegree = m_limelight.getdegRotationToTarget();
        double turnSpeed = turnToDegree * DriveConstants.kTurnFactor;
        m_drive.turn(turnSpeed);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        // Get everything in a safe starting state.
        m_drive.reset();
        super.initialize();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        boolean targetVisible = m_limelight.getIsTargetFound();
        boolean withinTolerance = Math.abs(m_limelight.getdegRotationToTarget()) <= 2.5;
        // End this Command if we reached our setpoint OR we don't have a target visible
        return !targetVisible || withinTolerance;
    }
}
