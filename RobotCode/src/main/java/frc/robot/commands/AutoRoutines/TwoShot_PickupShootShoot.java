// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import java.sql.PreparedStatement;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.LoadCargo;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.ShootTwiceAuto;
import frc.robot.commands.TurnToDegreeGyro;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// Three shot auto
public class TwoShot_PickupShootShoot extends SequentialCommandGroup {
        public TwoShot_PickupShootShoot(
                        Limelight limelight,
                        GTADrive drive,
                        Intake intake,
                        Conveyor conveyor,
                        Feeder feeder,
                        Shooter shooter,
                        IntakeCover intakeCover) {

                Command prepareSecondPickup = new SequentialCommandGroup(
                                new InstantCommand(() -> shooter.setSetpoint(-3850)),
                                new LoadCargo(intake, intakeCover, conveyor, feeder, shooter));

                Command driveAndPickupSecond = new SequentialCommandGroup(
                                new DriveStraight(40, drive),
                                new LoadCargoAuto(intake, conveyor, feeder, shooter, intakeCover),
                                new InstantCommand(() -> intakeCover.closeIntake()),
                                new WaitCommand(.5));

                // Command driveAndPickupSecond = new SequentialCommandGroup(
                // new DriveStraight(40, drive),
                // new LoadCargoAuto(intake, conveyor, feeder, shooter, intakeCover));

                Command shootFirstAndSecond = new SequentialCommandGroup(
                                new ShootAndWaitAuto(limelight, drive, intake, conveyor, feeder, shooter),
                                new TurnToDegreeGyro(-1 * drive.getHeading(), drive));

                // Command shootFirstAndSecond = new ShootTwiceAuto(() -> -3850, intake, feeder,
                // shooter, conveyor);
                Command prepareToPickupOpponentCargo = new SequentialCommandGroup(
                                new InstantCommand(() -> shooter.setSetpoint(-3850)),
                                new LoadCargo(intake, intakeCover, conveyor, feeder, shooter));

                Command pickupOpponentCargo = new SequentialCommandGroup(
                                new TurnToDegreeGyro(100, drive),
                                prepareToPickupOpponentCargo,
                                new DriveStraight(40, drive),
                                new LoadCargoAuto(intake, conveyor, feeder, shooter, intakeCover));

                Command shootAwayOpponentCargo = new SequentialCommandGroup(
                                new TurnToDegreeGyro(50, drive),
                                new DriveStraight(-20, drive),
                                new ShootAuto(() -> -2000, intake, feeder, shooter, conveyor));

                Command stop = new InstantCommand(() -> {
                        drive.drive(0, 0);
                        shooter.stopShooter();
                });

                addCommands(
                                prepareSecondPickup, driveAndPickupSecond, shootFirstAndSecond,
                                pickupOpponentCargo, shootAwayOpponentCargo,
                                stop);
        }
}
