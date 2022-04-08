// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.LoadCargo;
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
public class FourShot_PickupShootShoot extends SequentialCommandGroup {
    public FourShot_PickupShootShoot(
            Limelight limelight,
            GTADrive drive,
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter,
            IntakeCover intakeCover) {

        Command prepareSecondPickup = new LoadCargo(intake, intakeCover, conveyor, feeder, shooter);
        Command driveAndPickupSecond = new SequentialCommandGroup(
                new DriveStraight(40, drive),
                new LoadCargoAuto(intake, conveyor, feeder, shooter, intakeCover));
        Command shootFirstAndSecond = new ShootTwiceAuto(() -> -3850, intake, feeder, shooter, conveyor);
        Command prepareToDriveToFourthAndFifth = new TurnToDegreeGyro(-90, drive);
        Command driveToFourthAndFifth = new DriveStraight(224, drive);
        Command driveBackToShootFourthAndFifth = new SequentialCommandGroup(
                new DriveStraight(-165, drive),
                new TurnToDegreeGyro(40, drive));
        Command shootFourthAndFifth = new ShootAndWaitAuto(limelight, drive, intake, conveyor, feeder, shooter);

        Command stop = new InstantCommand(() -> {
            drive.drive(0, 0);
            shooter.stopShooter();
        });

        addCommands(
                prepareSecondPickup, driveAndPickupSecond, shootFirstAndSecond,
                prepareToDriveToFourthAndFifth, driveToFourthAndFifth,
                driveBackToShootFourthAndFifth, shootFourthAndFifth,
                stop);
    }
}
