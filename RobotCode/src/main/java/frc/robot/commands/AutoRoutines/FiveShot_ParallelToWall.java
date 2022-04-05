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

// Five shot auto
public class FiveShot_ParallelToWall extends SequentialCommandGroup {
    public FiveShot_ParallelToWall(
            Limelight limelight,
            GTADrive drive,
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter,
            IntakeCover intakeCover) {

        Command shootFirstThree = new ThreeShot_ParallelToWall(limelight, drive, intake, conveyor, feeder, shooter,
                intakeCover);

        Command prepareFourthAndFifthPickup = new LoadCargo(intake, intakeCover, conveyor, feeder, shooter);
        Command driveAndPickupFourthAndFifth = new SequentialCommandGroup(
                new TurnToDegreeGyro(35, drive),
                new DriveStraight(140, drive),
                prepareFourthAndFifthPickup, // Start running intake while we drive over
                new TurnToDegreeGyro(-40, drive),
                new DriveStraight(20, drive));
        Command driveToShootFourthFifth = new SequentialCommandGroup(
                new DriveStraight(-200, drive),
                new TurnToDegreeGyro(40, drive));
        Command shootFourthAndFifth = new ShootAndWaitAuto(limelight, drive, intake, conveyor, feeder, shooter);
        Command stop = new InstantCommand(() -> {
            drive.drive(0, 0);
            shooter.stopShooter();
        });

        addCommands(
                shootFirstThree,
                driveAndPickupFourthAndFifth,
                driveToShootFourthFifth,
                shootFourthAndFifth,
                stop);
    }
}
