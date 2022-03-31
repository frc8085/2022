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
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.GetSetpointFromDistance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class AutoSetpointWithLimelight extends CommandBase {
    private final GTADrive m_drive;
    private final Limelight m_limelight;
    private final Shooter m_shooter;
    private double setpoint;

    public AutoSetpointWithLimelight(GTADrive drive, Shooter shooter, Limelight limelight) {
        // Require the drive and limelight
        m_drive = drive;
        m_limelight = limelight;
        m_shooter = shooter;
        double distance = m_limelight.getDistanceToTarget();
        setpoint = GetSetpointFromDistance.setpointFromDistance(distance);
        addRequirements(m_drive, m_limelight);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        // Get everything in a safe starting state.
        super.initialize();
        m_drive.reset();
        m_shooter.setSetpoint(setpoint);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        boolean atSetpoint = m_shooter.atSetpoint();
        return atSetpoint;
    }
}
