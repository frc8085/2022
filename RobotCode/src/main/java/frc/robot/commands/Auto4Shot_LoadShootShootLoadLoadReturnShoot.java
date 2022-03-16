// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

/** Routine: Load second, shoot first and second */
public class Auto4Shot_LoadShootShootLoadLoadReturnShoot extends SequentialCommandGroup {
    private final Trajectory m_constrainedTrajectorySecond = Trajectories.kPickupSecond;
    private final String trajectoryJSONSecond = "paths/1-pickupSecond.wpilib.json";

    private final Trajectory m_constrainedTrajectoryThird = Trajectories.kPickupThird;
    private final String trajectoryJSONThird = "paths/2-pickupThird.wpilib.json";

    private final Trajectory m_constrainedTrajectoryFourth = Trajectories.kPickupFourth;
    private final String trajectoryJSONFourth = "paths/3-pickupFourth.wpilib.json";

    private final Trajectory m_constrainedTrajectoryReturn = Trajectories.kReturn;
    private final String trajectoryJSONReturn = "paths/4-return.wpilib.json";

    private final Command driveToSecond;
    private final Command driveToThird;
    private final Command driveToFourth;
    private final Command driveReturn;

    public Auto4Shot_LoadShootShootLoadLoadReturnShoot(
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
            driveToFourth = new AutoPathFromJSON(drive, trajectoryJSONFourth);
            driveReturn = new AutoPathFromJSON(drive, trajectoryJSONReturn);

        } else {
            driveToSecond = new AutoPath(drive, m_constrainedTrajectorySecond);
            driveToThird = new AutoPath(drive, m_constrainedTrajectoryThird);
            driveToFourth = new AutoPath(drive, m_constrainedTrajectoryFourth);
            driveReturn = new AutoPath(drive, m_constrainedTrajectoryReturn);

        }

        // Both of these shots are from the 2nd cargo pickup
        // TODO: Figure out the correct setpoint at the Second Cargo pickup location
        int setPointatSecondCargo = kShooterTargetRPM[kTargetBumpedTBD];
        Command prepareToShootAtSecondPickup = new InstantCommand(() -> shooter.setSetpoint(setPointatSecondCargo));

        Command shootFirst = new ShootAuto(setPointatSecondCargo, intake, conveyor, feeder, shooter);
        Command shootSecond = new ShootAuto(setPointatSecondCargo, intake, conveyor, feeder, shooter);

        // Load cargos
        Command pickupSecond = new PickupAuto(intake, intakeCover, conveyor, feeder, shooter);
        Command pickupThird = new PickupAuto(intake, intakeCover, conveyor, feeder, shooter);
        Command pickupFourth = new PickupAuto(intake, intakeCover, conveyor, feeder, shooter);

        // Prepare to shoot at return endpoint
        // TODO: Figure out the correct setpoint
        int setpointAtReturn = kShooterTargetRPM[kTargetBumpedTBD];
        Command prepareToShootAfterReturn = new InstantCommand(() -> shooter.setSetpoint(setpointAtReturn));

        // Both of these shots are from the return endpoint
        Command shootThird = new ShootAuto(setpointAtReturn, intake, conveyor, feeder, shooter);
        Command shootFourth = new ShootAuto(setpointAtReturn, intake, conveyor, feeder, shooter);

        addCommands(
                driveToSecond,
                parallel(prepareToShootAtSecondPickup, pickupSecond), // Prepare for first two shots while pickup up
                                                                      // second cargo
                shootFirst,
                shootSecond,
                driveToThird,
                pickupThird,
                driveToFourth,
                pickupFourth,
                parallel(prepareToShootAfterReturn, driveReturn), // Prepare for final shots while driving
                shootThird,
                shootFourth);
    }
}