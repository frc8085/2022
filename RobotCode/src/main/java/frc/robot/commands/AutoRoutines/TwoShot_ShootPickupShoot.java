// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoAimWithLimelight;
import frc.robot.commands.AutoSetpointShot;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.HoldCargo;
import frc.robot.commands.LoadCargo;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToDegreeGyro;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// Two Shot Auto. ???? What's the starting position?

public class TwoShot_ShootPickupShoot extends SequentialCommandGroup {
        public TwoShot_ShootPickupShoot(
                        Limelight limelight,
                        GTADrive drive,
                        Intake intake,
                        Conveyor conveyor,
                        Feeder feeder,
                        Shooter shooter,
                        IntakeCover intakeCover) {

                // The ShootAndWaitAuto can take a setpoint OR the limelight
                // Here we shoot first so use setpoint (limelight can't see the target)
                Command shootFirst = new ShootAndWaitAuto(-2800, drive, intake, conveyor, feeder, shooter);
                Command parepareAndPickupSecond = new SequentialCommandGroup(
                                new LoadCargo(intake, intakeCover, conveyor, feeder, shooter),
                                new DriveStraight(40, drive));
                Command shootSecond = new ShootAndWaitAuto(limelight, drive, intake, conveyor, feeder, shooter);
                Command stop = new InstantCommand(() -> {
                        drive.drive(0, 0);
                        shooter.stopShooter();
                });

                addCommands(
                                shootFirst,
                                parepareAndPickupSecond, shootSecond,
                                stop);
        }
}
