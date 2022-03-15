// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kDriveKinematics;
import static frc.robot.Constants.DriveConstants.kPDriveVel;
import static frc.robot.Constants.DriveConstants.kRamseteB;
import static frc.robot.Constants.DriveConstants.kRamseteZeta;
import static frc.robot.Constants.DriveConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.DriveConstants.ksVolts;
import static frc.robot.Constants.DriveConstants.kvVoltSecondsPerMeter;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GTADrive;

public class AutoPathFromJSON extends SequentialCommandGroup {
        private final GTADrive m_drive;
        private final Trajectory m_autoTrajectory;

        /**
         * Create a new path from PathWaver JSON output files. Follow it using odometry
         */
        public AutoPathFromJSON(GTADrive drive, String trajectoryJSON) {
                m_drive = drive;
                // m_autoTrajectory = autoTrajectory;
                addRequirements(drive);

                // The trajectory to follow. All units in meters.
                Trajectory newTrajectory = new Trajectory();

                try {
                        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        newTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
                }

                m_autoTrajectory = newTrajectory;

                // The subcommands for this AutoPath
                Command m_resetOdometry = new InstantCommand(
                                () -> m_drive.resetOdometry(m_autoTrajectory.getInitialPose()));
                RamseteCommand m_autoCommand = new RamseteCommand(
                                m_autoTrajectory,
                                m_drive::getPose,
                                new RamseteController(kRamseteB, kRamseteZeta),
                                new SimpleMotorFeedforward(
                                                ksVolts,
                                                kvVoltSecondsPerMeter,
                                                kaVoltSecondsSquaredPerMeter),
                                kDriveKinematics,
                                m_drive::getWheelSpeeds,
                                new PIDController(kPDriveVel, 0, 0),
                                new PIDController(kPDriveVel, 0, 0),
                                // RamseteCommand passes volts to the callback
                                m_drive::tankDriveVolts,
                                m_drive);

                addCommands(m_resetOdometry, m_autoCommand);
        }
}