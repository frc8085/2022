// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * Automatically determine the correct shooting setpoint
 * 1. Use Limelight to estimate the area of the target
 * 2. Estimate our distance to the target based on the area on the Limelight
 * image
 * 3. Calculate the setpoint based on estimated distance
 * 4. Get up to speed
 */
public class AutoSetpointWithLimelight extends CommandBase {
    private final Shooter m_shooter;
    private final Limelight m_limelight;
    private double setpoint;
    private NetworkTableEntry setpointForTarget;

    public AutoSetpointWithLimelight(Shooter shooter, Limelight limelight) {
        // Require the drive and limelight
        m_shooter = shooter;
        m_limelight = limelight;
        addRequirements(m_shooter, m_limelight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = m_limelight.getDistanceToTarget();
        // m_shooter.setSetpointFromDistance(distance);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // End when we're at setpoint, unless setpoint is 0
        // return setpoint > 0 && m_shooter.atSetpoint();
        return false;
    }
}
