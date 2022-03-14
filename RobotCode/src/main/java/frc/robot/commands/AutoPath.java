// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.GTADrive;

public class AutoPath extends CommandBase {
    private static final double ramseteB = 2;
    private static final double ramseteZeta = 0.7;
    private final GTADrive drive;
    private DifferentialDriveKinematics driveKinematics;
    private final Trajectory autoTrajectory;
    private final RamseteController ramseteController = new RamseteController(ramseteB, ramseteZeta);
    private final Timer timer = new Timer();

    /**
     * Create a new path with constraints. Drive along the path using odometry
     */
    public AutoPath(GTADrive drive, double startVelocityMetersPerSec,
            List<Pose2d> waypoints, double endVelocityMetersPerSec, boolean reversed,
            List<TrajectoryConstraint> constraints) {

        this.drive = drive;
        addRequirements(drive);

        // Set up trajectory configuration
        driveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(kTrackWidthInches));

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        ksVolts,
                        kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter),
                driveKinematics,
                10); // Max Voltage. TODO: Make this a constant.

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(driveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // Now create the trajectory to follow. All units in meters.
        autoTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RamseteCommand ramseteCommand = new RamseteCommand(
                autoTrajectory,
                drive::getPose,
                new RamseteController(kRamseteB, kRamseteZeta),
                new SimpleMotorFeedforward(
                        ksVolts,
                        kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter),
                driveKinematics,
                drive::getWheelSpeeds,
                new PIDController(kPDriveVel, 0, 0),
                new PIDController(kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drive::tankDriveVolts,
                drive);

        State setpoint = autoTrajectory.sample(timer.get());
        ChassisSpeeds chassisSpeeds = ramseteController.calculate(drive.getPose(), setpoint);
        DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(
                chassisSpeeds);
        drive.drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(autoTrajectory.getTotalTimeSeconds());
    }

    public double getDuration() {
        return autoTrajectory.getTotalTimeSeconds();
    }
}