// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
 * Routine: Load second. Shoot first cargo while loading third. Shoot second and
 * third.
 */
public class Auto3Shot_LoadShootWhileLoadLoadShootShoot extends SequentialCommandGroup {
        private final Trajectory m_constrainedTrajectorySecond = Trajectories.kPickupSecond;
        private final String trajectoryJSONSecond = "paths/1-pickupSecond.wpilib.json";

        private final Trajectory m_constrainedTrajectoryThird = Trajectories.kPickupThird;
        private final String trajectoryJSONThird = "paths/2-pickupThird.wpilib.json";

        private final Command driveToSecond;
        private final Command driveToThird;

        private final double m_intakeDurationSeconds = 2;

        public Auto3Shot_LoadShootWhileLoadLoadShootShoot(
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
                        driveToSecond = new AutoPathFromJSON(drive, trajectoryJSONSecond);
                        driveToThird = new AutoPathFromJSON(drive, trajectoryJSONThird);
                } else {
                        driveToSecond = new AutoPath(drive, m_constrainedTrajectorySecond);
                        driveToThird = new AutoPath(drive, m_constrainedTrajectoryThird);
                }

                // Set the setpoint for the shot at our destination
                // TODO: Figure out setpoint for this routine

                int setpointAtDestination = kShooterTargetRPM[kTargetBumpedTBD];

                Command prepareShot = new InstantCommand(
                                () -> shooter.setSetpoint(setpointAtDestination));
                Command pickupSecond = new PickupAuto(intake, intakeCover, conveyor, feeder, shooter);
                Command shootFirst = new ShootAuto(kTargetBumpedTBD, intake, conveyor, feeder, shooter);
                Command pickupThird = new PickupAuto(intake, intakeCover, conveyor, feeder, shooter);
                Command shootSecond = new ShootAuto(kTargetBumpedTBD, intake, conveyor, feeder, shooter);
                Command shootThird = new ShootAuto(kTargetBumpedTBD, intake, conveyor, feeder, shooter);

                addCommands(driveToSecond,
                                pickupSecond, // Load second cargo
                                parallel(prepareShot, driveToThird), // Drive while preparing to shoot
                                shootFirst, // Shot first cargo before we load third
                                pickupThird, // Load third
                                shootSecond, // Shoot second and third cargo
                                shootThird);
        }
}