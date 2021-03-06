// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.GTADrive;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to run a simple PID loop that is only enabled while t
 * is command is running. The input is the averaged values of the left and right
 * encoders.
 */
public class TurnToDegree extends PIDCommand {
    private final GTADrive m_drive;

    static double kP = 0.1;
    static double kI = 0;
    static double kD = 0.001;

    /**
     * Create a new TurnToDegree command.
     *
     * @param distance The distance to drive (inches)
     */
    public TurnToDegree(double degree, GTADrive drive) {
        super(
                new PIDController(kP, kI, kD),
                drive::getTurnedInches,
                degree * kTurnFactor, // Convert degrees to distance
                d -> drive.turn(d));

        m_drive = drive;
        addRequirements(m_drive);

        getController().setTolerance(kAutoPositionTolerance);
    }

    @Override
    public void execute() {
        super.execute();
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
        return getController().atSetpoint();
    }
}
