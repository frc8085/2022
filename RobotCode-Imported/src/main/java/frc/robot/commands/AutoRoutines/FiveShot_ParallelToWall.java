// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

                Command prepareShot = new SequentialCommandGroup(
                                new InstantCommand(() -> shooter.setSetpoint(-3850)),
                                new InstantCommand(() -> intakeCover.closeIntake()),
                                new WaitCommand(.5));

                Command driveBack = new SequentialCommandGroup(
                                new DriveStraight(40, drive),
                                new WaitCommand(.5));

                Command shootFirst = new SequentialCommandGroup(
                                new ShootAndWaitAuto(limelight, drive, intake, conveyor, feeder, shooter),
                                new TurnToDegreeGyro(-1 * drive.getHeading(), drive));

                Command driveBackAgain = new SequentialCommandGroup(
                                new DriveStraight(40, drive),
                                new WaitCommand(.5));

                Command stop = new InstantCommand(() -> {
                        drive.drive(0, 0);
                        shooter.stopShooter();
                });

                addCommands(
                                prepareShot, driveBack,
                                shootFirst, driveBackAgain,
                                stop);
        }
}
