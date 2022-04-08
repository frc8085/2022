// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.LoadCargo;
import frc.robot.commands.AutoRoutines.LoadCargoAuto;
import frc.robot.commands.Shoot;
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
public class ThreeShot_ParallelToWall extends SequentialCommandGroup {
        private double headingBeforeAutoAim;
        private double headingAfterAutoAim;

        public ThreeShot_ParallelToWall(
                        Limelight limelight,
                        GTADrive drive,
                        Intake intake,
                        Conveyor conveyor,
                        Feeder feeder,
                        Shooter shooter,
                        IntakeCover intakeCover) {

                Command shootFirstTwo = new TwoShot_PickupShootShoot(limelight, drive, intake, conveyor, feeder,
                                shooter, intakeCover);
                Command prepareThirdPickup = new LoadCargo(intake, intakeCover, conveyor, feeder, shooter);
                Command driveAndPickupThird = new SequentialCommandGroup(
                                new DriveStraight(-60, drive),
                                new TurnToDegreeGyro(75, drive),
                                new DriveStraight(80, drive),
                                new LoadCargoAuto(intake, conveyor, feeder, shooter, intakeCover));

                Command turnToShootThird = new TurnToDegreeGyro(-30, drive);
                Command shootThird = new SequentialCommandGroup(
                                new InstantCommand(() -> headingBeforeAutoAim = drive.getHeading()),
                                new ShootAndWaitAuto(limelight, drive, intake, conveyor, feeder, shooter),
                                new InstantCommand(() -> headingAfterAutoAim = drive.getHeading())

                );
                Command stop = new InstantCommand(() -> {
                        drive.drive(0, 0);
                        shooter.stopShooter();
                });

                addCommands(
                                shootFirstTwo,
                                prepareThirdPickup, driveAndPickupThird,
                                // turnToShootThird,
                                shootThird,
                                stop);
        }
}
