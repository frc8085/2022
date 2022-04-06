// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.LoadCargo;
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
        public ThreeShot_ParallelToWall(
                        Limelight limelight,
                        GTADrive drive,
                        Intake intake,
                        Conveyor conveyor,
                        Feeder feeder,
                        Shooter shooter,
                        IntakeCover intakeCover) {

                Command prepareSecondPickup = new LoadCargo(intake, intakeCover, conveyor, feeder, shooter);
                Command driveAndPickupSecond = new DriveStraight(40, drive);
                Command shootFirstAndSecond = new ShootAndWaitAuto(limelight, drive, intake, conveyor, feeder, shooter);
                Command prepareThirdPickup = new LoadCargo(intake, intakeCover, conveyor, feeder, shooter);
                Command driveAndPickupThird = new SequentialCommandGroup(
                                new DriveStraight(-60, drive),
                                new TurnToDegreeGyro(75, drive),
                                new DriveStraight(77, drive));
                Command turnToShootThird = new TurnToDegreeGyro(-20, drive);
                Command shootThird = new ShootAndWaitAuto(limelight, drive, intake, conveyor, feeder, shooter);
                Command stop = new InstantCommand(() -> {
                        drive.drive(0, 0);
                        shooter.stopShooter();
                });

                addCommands(
                                prepareSecondPickup, driveAndPickupSecond, shootFirstAndSecond,
                                prepareThirdPickup, driveAndPickupThird, turnToShootThird, shootThird,
                                stop);
        }
}