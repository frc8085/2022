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
 * local PID controller to run a simple PID loop that is only enabled while
 * this command is running. The input is the averaged values of the left and
 * right encoders.
 */
public class DriveStraight extends PIDCommand {
    private final GTADrive m_drive;

    // worked at kP=.01 kD=.001
    static double kP = 0.045;
    static double kI = 0;
    static double kD = 0.0025;

    /**
     * Create a new DriveStraight command.
     *
     * @param distance The distance to drive (inches)
     */
    public DriveStraight(double distance, GTADrive drive) {
        super(
                new PIDController(kP, kI, kD),
                // Close loop on distance
                drive::getDistance,
                // Set reference to target distance
                distance,
                // Pipe output to drive the robot
                output -> drive.drive(output, output));

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
