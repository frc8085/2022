// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

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
import frc.robot.Constants;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.GTADrive;

public class AutoPath extends CommandBase {
    private static final double ramseteB = 2;
    private static final double ramseteZeta = 0.7;
    private final GTADrive drive;
    private DifferentialDriveKinematics m_DriveKinematics;
    private final Trajectory trajectory;
    private final RamseteController controller = new RamseteController(ramseteB, ramseteZeta);
    private final Timer timer = new Timer();

    /**
     * Create a new path with constraints. Drive along the path using odometry
     */
    public AutoPath(GTADrive drive, double startVelocityMetersPerSec,
            List<Pose2d> waypoints, double endVelocityMetersPerSec, boolean reversed,
            List<TrajectoryConstraint> constraints) {

        this.drive = drive;
        addRequirements(drive);

        // Select max velocity & acceleration
        double maxVoltage, maxVelocityMetersPerSec, maxAccelerationMetersPerSec2,
                maxCentripetalAccelerationMetersPerSec2;

        maxVoltage = 10.0;
        maxVelocityMetersPerSec = Units.inchesToMeters(210.0);
        maxAccelerationMetersPerSec2 = kMaxAccelerationMetersPerSecondSquared;
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(75.0);

        // Set up trajectory configuration
        m_DriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(kTrackWidthInches));

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        ksVolts,
                        kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter),
                m_DriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(m_DriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        TrajectoryConfig config = new TrajectoryConfig(maxVelocityMetersPerSec,
                maxAccelerationMetersPerSec2).setKinematics(kinematics)
                        .addConstraints(constraints)
                        .setStartVelocity(startVelocityMetersPerSec)
                        .setEndVelocity(endVelocityMetersPerSec).setReversed(reversed);
        if (Units.inchesToMeters(kaVoltSecondsSquaredPerInch) != 0) {
            config.addConstraint(new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(

                            drive.getKs(), drive.getKv(),
                            drive.getKa()),
                    kinematics, maxVoltage));
        }

        // Generate trajectory
        Trajectory generatedTrajectory;
        try {
            generatedTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
        } catch (TrajectoryGenerationException exception) {
            generatedTrajectory = new Trajectory();
        }
        trajectory = generatedTrajectory;
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
        State setpoint = trajectory.sample(timer.get());
        Logger.getInstance().recordOutput("Odometry/ProfileSetpoint",
                new double[] { setpoint.poseMeters.getX(), setpoint.poseMeters.getY(),
                        setpoint.poseMeters.getRotation().getRadians() });
        ChassisSpeeds chassisSpeeds = controller.calculate(drive.getPose(), setpoint);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        drive.driveVelocity(wheelSpeeds.leftMetersPerSecond,
                wheelSpeeds.rightMetersPerSecond);
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
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public double getDuration() {
        return trajectory.getTotalTimeSeconds();
    }
}