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
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class AutoAimWithLimelight extends CommandBase {
    private final GTADrive m_drivetrain;
    private final Limelight m_limelight;

    static double kP = 0.01;
    static double kI = 0;
    static double kD = 0.001;

    public AutoAimWithLimelight(GTADrive drivetrain, Limelight limelight) {
        // Require the drive and limelight
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        addRequirements(m_drivetrain, m_limelight);
    }

    @Override
    public void execute() {
        super.execute();
        double turnToDegree = m_limelight.getdegRotationToTarget();
        double turnSpeed = turnToDegree * DriveConstants.kTurnFactor;
        m_drivetrain.turn(turnSpeed);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        // Get everything in a safe starting state.
        m_drivetrain.reset();
        super.initialize();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(m_limelight.getdegRotationToTarget()) <= 2.5;
    }
}