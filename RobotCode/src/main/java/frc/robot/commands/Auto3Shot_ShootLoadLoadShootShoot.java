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

/**
 * Routine: Shoot first cargo. Load second and third. Shoot second and third.
 */
public class Auto3Shot_ShootLoadLoadShootShoot extends SequentialCommandGroup {
        private final Trajectory m_constrainedTrajectorySecond = Trajectories.kPickupSecond;
        private final String trajectoryJSONSecond = "paths/1-pickupSecond.wpilib.json";

        private final Trajectory m_constrainedTrajectoryThird = Trajectories.kPickupThird;
        private final String trajectoryJSONThird = "paths/2-pickupThird.wpilib.json";

        private final Command driveAndLoadSecond;
        private final Command driveAndLoadThird;

        public Auto3Shot_ShootLoadLoadShootShoot(
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
                        driveAndLoadSecond = new DriveAndLoadAuto(trajectoryJSONSecond, drive, intake, conveyor,
                                        feeder,
                                        shooter, intakeCover);
                        driveAndLoadThird = new DriveAndLoadAuto(trajectoryJSONThird, drive, intake, conveyor,
                                        feeder,
                                        shooter, intakeCover);
                } else {
                        driveAndLoadSecond = new DriveAndLoadAuto(m_constrainedTrajectorySecond, drive, intake,
                                        conveyor,
                                        feeder,
                                        shooter, intakeCover);
                        driveAndLoadThird = new DriveAndLoadAuto(m_constrainedTrajectoryThird, drive, intake, conveyor,
                                        feeder,
                                        shooter, intakeCover);
                }

                // TODO: Definet he setpoint for starting point
                int setpointAtStart = kShooterTargetRPM[kTargetBumpedNear];
                Command shootFirst = new ShootAuto(setpointAtStart, intake, conveyor, feeder, shooter);

                // TODO: Define the setpoint for third cargo location
                int setpointAtThirdCargo = kShooterTargetRPM[kTargetBumpedTBD];

                Command shootSecond = new ShootAuto(setpointAtThirdCargo, intake, conveyor, feeder, shooter);
                Command shootThird = new ShootAuto(setpointAtThirdCargo, intake, conveyor, feeder, shooter);

                addCommands(shootFirst, driveAndLoadSecond, driveAndLoadThird, shootSecond, shootThird);
        }
}