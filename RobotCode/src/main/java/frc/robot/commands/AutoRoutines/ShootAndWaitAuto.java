// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoAimWithLimelight;
import frc.robot.commands.AutoSetpointShot;
import frc.robot.commands.HoldCargo;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// In auto, we have to make sure we've completed the shots before we drive away
public class ShootAndWaitAuto extends SequentialCommandGroup {
    private double shootDurationSecs = 0.6;

    // Uses the limelight and tries to shoot twice
    public ShootAndWaitAuto(
            Limelight limelight,
            GTADrive drive,
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter) {

        addCommands(
                new HoldCargo(intake, conveyor, feeder),
                new AutoAimWithLimelight(drive, limelight),
                new AutoAimWithLimelight(drive, limelight),
                new AutoAimWithLimelight(drive, limelight),
                new AutoSetpointShot(drive, limelight, intake, feeder, shooter, conveyor),
                new WaitCommand(shootDurationSecs * 2));
    }

    // Doesn't use the limelight, and only shoots once
    public ShootAndWaitAuto(
            double setpoint,
            GTADrive drive,
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter) {

        addCommands(
                new HoldCargo(intake, conveyor, feeder),
                new InstantCommand(() -> shooter.setSetpoint(setpoint)),
                new WaitUntilCommand(shooter::atSetpoint),
                new Shoot(intake, feeder, shooter, conveyor),
                new WaitCommand(shootDurationSecs));
    }
}
