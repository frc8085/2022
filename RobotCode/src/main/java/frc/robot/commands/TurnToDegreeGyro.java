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
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class TurnToDegreeGyro extends PIDCommand {
    private final GTADrive m_drivetrain;

    static double kP = 0.01;
    static double kI = 0;
    static double kD = 0.001;

    /**
     * Create a new TurnToDegreeGyro command.
     *
     * @param distance The distance to drive (inches)
     */
    public TurnToDegreeGyro(double degree, GTADrive drivetrain) {
        super(new PIDController(kP, kI, kD),
                // Close loop on heading
                drivetrain::getHeading,
                // Set reference to target
                degree,
                // Pipe output to turn robot
                output -> drivetrain.turn(output));

        // Require the drive
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);

        getController().setTolerance(kAutoGyroTolerance);
    }

    @Override
    public void execute() {
        super.execute();
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
        return getController().atSetpoint();
    }
}
