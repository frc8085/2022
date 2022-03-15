// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GTADrive;

public class AutoPath extends SequentialCommandGroup {
        private final GTADrive m_drive;
        private final Trajectory m_autoTrajectory;

        /**
         * Create a new auto path with constraints. Drive along the path using odometry.
         */
        public AutoPath(GTADrive drive, Trajectory autoTrajectory) {
                m_drive = drive;
                m_autoTrajectory = autoTrajectory;
                addRequirements(drive);

                // The subcommands for this AutoPath
                Command m_resetOdometry = new InstantCommand(
                                () -> m_drive.resetOdometry(m_autoTrajectory.getInitialPose()));

                RamseteCommand m_autoCommand = new RamseteCommand(
                                m_autoTrajectory, // trajectory
                                m_drive::getPose, // pose
                                new RamseteController(kRamseteB, kRamseteZeta), // controller
                                new SimpleMotorFeedforward( // feedforward
                                                ksVolts,
                                                kvVoltSecondsPerMeter,
                                                kaVoltSecondsSquaredPerMeter),
                                kDriveKinematics,
                                m_drive::getWheelSpeeds, // wheelSpeeds
                                new PIDController(kPDriveVel, 0, 0), // leftController
                                new PIDController(kPDriveVel, 0, 0), // rightController
                                // RamseteCommand passes volts to the callback
                                m_drive::tankDriveVolts, // outputVolts
                                m_drive);

                addCommands(m_resetOdometry, m_autoCommand);
        }
}