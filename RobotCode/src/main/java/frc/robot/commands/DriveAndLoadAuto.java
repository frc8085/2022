// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Shooter;

/** The pickup path to load the second cargo. Does NOT include shots */
public class DriveAndLoadAuto extends SequentialCommandGroup {
        private final double m_intakeDurationSeconds = 2;

        /**
         * Create a new auto path with constraints. Drive along the path using odometry.
         */
        public DriveAndLoadAuto(
                        Trajectory trajectory,
                        GTADrive drive,
                        Intake intake,
                        Conveyor conveyor,
                        Feeder feeder,
                        Shooter shooter,
                        IntakeCover intakeCover) {

                addRequirements(drive);

                // The Auto path to follow for this trajectory
                Command driveToCargo = new AutoPath(drive, trajectory);
                Command pickupCargo = sequence(
                                new WaitCommand(m_intakeDurationSeconds)
                                                .deadlineWith(new LoadCargo(intake, intakeCover, conveyor, feeder,
                                                                shooter)),
                                new HoldCargo(intake, conveyor, feeder));

                // Drive to the second cargo and pick it up
                addCommands(driveToCargo, pickupCargo);
        }
}