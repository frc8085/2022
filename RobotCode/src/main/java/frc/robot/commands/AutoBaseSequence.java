// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Shooter;

// Run feeder and conveyor in the same direction at a set speed.
// Make sure that the feeder is not running
public class AutoBaseSequence extends SequentialCommandGroup {
        public AutoBaseSequence(
                        int shootingMode1,
                        double driveDistance1,
                        boolean pickUp1,
                        double driveDistance2,
                        int shootingMode2,
                        double turnDegrees1,
                        double driveDistance3,
                        boolean pickUp2,
                        double driveDistance4,
                        int shootingMode3,
                        GTADrive drive,
                        Intake intake,
                        IntakeCover intakeCover,
                        Conveyor conveyor,
                        Feeder feeder,
                        Shooter shooter) {
                addCommands(
                                /* 1 */ new ShootAuto(kShooterTargetRPM[shootingMode1], intake,
                                                conveyor, feeder, shooter),
                                new WaitCommand(.25),
                                /* 2 */ new DriveStraight(driveDistance1, drive),
                                /* 3 */ !pickUp1 ? new InstantCommand()
                                                : new LoadCargo(intake, intakeCover, conveyor, feeder, shooter)
                                                                .andThen(new WaitCommand(2))
                                                                .andThen(new HoldCargo(intake, conveyor, feeder)),
                                /* 4 */ new DriveStraight(driveDistance2, drive),
                                /* 5 */ new ShootAuto(kShooterTargetRPM[shootingMode2], intake,
                                                conveyor, feeder, shooter),
                                /* 6 */ new TurnToDegreeGyro(turnDegrees1, drive),
                                /* 7 */ new DriveStraight(driveDistance3, drive),
                                /* 8 */ !pickUp2 ? new InstantCommand()
                                                : new LoadCargo(intake, intakeCover, conveyor, feeder, shooter)
                                                                .andThen(new WaitCommand(2))
                                                                .andThen(new HoldCargo(intake, conveyor, feeder)),
                                /*   */ new WaitCommand(2),
                                /*   */ new HoldCargo(intake, conveyor, feeder),
                                /* 9 */ new DriveStraight(driveDistance4, drive),
                                /* 10 */ new ShootAuto(kShooterTargetRPM[shootingMode3], intake,
                                                conveyor, feeder, shooter),
                                /* 11 */ new DriveStraight(0, drive)); // Force it to stop

                /// 305.66-116.17 (distance)
                // 188 (back)
                // shoot high
        }
}
