// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Trajectories;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.Trajectories.TrajectoryType;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Shooter;

/** Routine: Shoot first, load second, shoot second */
public class Auto2Shot_ShootLoadShoot extends SequentialCommandGroup {
    private final Trajectory m_constrainedTrajectory = Trajectories.kPickupSecond;
    private final String trajectoryJSON = "paths/1-pickupSecond.wpilib.json";
    private final Command driveAndLoadSecond;

    public Auto2Shot_ShootLoadShoot(
            TrajectoryType type,
            GTADrive drive,
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter,
            IntakeCover intakeCover) {

        // If we're using the exported pathweaver JSON, load it. Otherwise follow the
        // safety constrainted trajectory created in code
        if (type == TrajectoryType.PATHWEAVER) {
            driveAndLoadSecond = new DriveAndLoadAuto(trajectoryJSON, drive, intake, conveyor,
                    feeder,
                    shooter, intakeCover);
        } else {
            driveAndLoadSecond = new DriveAndLoadAuto(m_constrainedTrajectory, drive, intake, conveyor,
                    feeder,
                    shooter, intakeCover);
        }

        Command shootFirst = new ShootAuto(kShooterTargetRPM[kTargetBumpedNear], intake, conveyor, feeder, shooter);
        Command shootSecond = new ShootAuto(kShooterTargetRPM[kTargetBumpedTBD], intake, conveyor, feeder, shooter);

        addCommands(shootFirst, driveAndLoadSecond, shootSecond);
    }
}