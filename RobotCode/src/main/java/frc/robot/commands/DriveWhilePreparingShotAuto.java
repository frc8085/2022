// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Shooter;

/**
 * Drive to the cargo destination while preparing to shoot. DOES NOT load or
 * shoot the cargo.
 */
public class DriveWhilePreparingShotAuto extends SequentialCommandGroup {

        /**
         * Gets to setpoint and drives along path.
         */
        public DriveWhilePreparingShotAuto(
                        int setpoint,
                        Trajectory trajectory,
                        GTADrive drive,
                        Intake intake,
                        Conveyor conveyor,
                        Feeder feeder,
                        Shooter shooter,
                        IntakeCover intakeCover) {

                addRequirements(drive);

                // The setpoint for the shot at our destination
                Command setSetpoint = new InstantCommand(() -> shooter.setSetpoint(setpoint));
                // The Auto path to follow for this trajectory
                Command driveToCargo = new AutoPath(drive, trajectory);

                // Drive to the destination while preparing ot shoot.
                addCommands(parallel(setSetpoint, driveToCargo));
        }

        /** Alternative: Create an Auto path using Pathweaver exported JSON path file */
        public DriveWhilePreparingShotAuto(
                        int setpoint,
                        String trajectoryJSON,
                        GTADrive drive,
                        Intake intake,
                        Conveyor conveyor,
                        Feeder feeder,
                        Shooter shooter,
                        IntakeCover intakeCover) {

                addRequirements(drive);

                // The setpoint for the shot at our destination
                Command setSetpoint = new InstantCommand(() -> shooter.setSetpoint(setpoint));
                // The Auto path to follow for this trajectory
                Command driveToCargo = new AutoPathFromJSON(drive, trajectoryJSON);

                // Drive to the destination while preparing ot shoot.
                addCommands(parallel(setSetpoint, driveToCargo));
        }

}