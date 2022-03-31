// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Trajectories;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.List;

import frc.robot.Trajectories.TrajectoryType;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Shooter;

/** Routine: Load second, shoot first and second */
public class BasicRamsete extends SequentialCommandGroup {
    private final Trajectory trajectory = Trajectories.kBasic;
    private final Command driveAndLoadSecond;

    public BasicRamsete(
            GTADrive drive,
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter,
            IntakeCover intakeCover) {

        driveAndLoadSecond = new DriveAndLoadAuto(trajectory, drive, intake, conveyor,
                feeder, shooter, intakeCover);

        Command shootFirst = new ShootAuto(kShooterTargetRPM[kTargetBumpedTBD], intake, conveyor, feeder, shooter);
        Command shootSecond = new ShootAuto(kShooterTargetRPM[kTargetBumpedTBD], intake, conveyor, feeder, shooter);

        addCommands(shootFirst, driveAndLoadSecond, shootSecond);
    }
}