// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Limelight;

/**
 * Aim using the limelight degree rotation to target
 */
public class AutoAimWithLimelight extends CommandBase {
    private final GTADrive m_drive;
    private final Limelight m_limelight;

    // PID Constants
    static double kP = 0.01;
    static double kI = 0;
    static double kD = 0.001;
    private final PIDController pidController = new PIDController(kP, kI, kD);

    // Creates a MedianFilter with a window size of 5 samples
    MedianFilter filter = new MedianFilter(5);

    public AutoAimWithLimelight(GTADrive drive, Limelight limelight) {
        // Require the drive and limelight
        m_drive = drive;
        m_limelight = limelight;
        addRequirements(m_drive, m_limelight);
        pidController.setTolerance(2.5);
    }

    @Override
    public void execute() {
        super.execute();
        // Use the median from the last 5 readings
        // We do this because the input can be erratic
        // Median is more robust than average
        double turnToDegree = filter.calculate(m_limelight.getdegRotationToTarget());
        System.out.println("DEGREE " + turnToDegree);

        double turnSpeed = -pidController.calculate(turnToDegree * kTurnFactor);
        m_drive.turn(turnSpeed);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        // Get everything in a safe starting state.
        super.initialize();
        m_drive.reset();
        pidController.reset();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        boolean targetVisible = m_limelight.getIsTargetFound();
        // End this Command if we reached our setpoint OR we lost the target
        return !targetVisible || pidController.atSetpoint();
    }
}
