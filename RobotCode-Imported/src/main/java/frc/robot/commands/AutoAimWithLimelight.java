// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * Aim using the limelight degree rotation to target
 */
public class AutoAimWithLimelight extends CommandBase {
    private final GTADrive m_drive;
    private final Limelight m_limelight;
    private final Shooter m_shooter;

    // Creates a MedianFilter with a window size of 5 samples
    MedianFilter filter = new MedianFilter(5);
    MedianFilter setpointFilter = new MedianFilter(5);

    public AutoAimWithLimelight(GTADrive drive, Limelight limelight, Shooter shooter) {
        // Require the drive and limelight
        m_drive = drive;
        m_limelight = limelight;
        m_shooter = shooter;
        addRequirements(m_drive, m_limelight, m_shooter);
    }

    @Override
    public void execute() {
        super.execute();

        double turnToDegree = m_limelight.getdegRotationToTarget();

        System.out.println("DEGREE " + turnToDegree);
        // Use the median from the last 5 readings
        // We do this because the input can be erratic
        // Median is more robust than average
        double medianDegree = filter.calculate(turnToDegree);
        double turnSpeed = medianDegree * DriveConstants.kTurnFactor;
        m_drive.turn(turnSpeed);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        // Get everything in a safe starting state.
        m_drive.reset();

        // Any time we run the auto-aim, start by running the shooter
        // WARNING: Operator needs to manually stop the shooter (press X)
        m_shooter.setSetpoint(-3550);
        super.initialize();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        boolean targetVisible = m_limelight.getIsTargetFound();
        double medianRotation = setpointFilter.calculate(m_limelight.getdegRotationToTarget());
        boolean withinTolerance = Math.abs(medianRotation) <= 2.5;
        // End this Command if we reached our setpoint OR we don't have a target visible
        return !targetVisible || withinTolerance;
    }
}
