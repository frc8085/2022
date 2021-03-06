// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Shooter;

/**
 * Automatically turn to the target, determine the shooting setpoint, get to it,
 * and shoot.
 * 1. Use Limelight to turn to the target
 * 2. Use Limelight to get up to shooting speed based on target disatnce
 * 3. Turn to the target, get up to speed and shoot
 */

public class AutoSetpointShot extends SequentialCommandGroup {
    public AutoSetpointShot(
            GTADrive drive,
            Limelight limelight,
            Intake intake,
            Feeder feeder,
            Shooter shooter,
            Conveyor conveyor) {
        addCommands(
                new InstantCommand(() -> shooter.setSetpointFromDistance(limelight::getDistanceToTarget)),
                new InstantCommand(conveyor::runConveyor, conveyor),
                new WaitUntilCommand(shooter::atSetpoint),
                new Shoot(intake, feeder, shooter, conveyor),
                new WaitUntilCommand(shooter::atSetpoint),
                new Shoot(intake, feeder, shooter, conveyor),
                new WaitUntilCommand(shooter::atSetpoint),
                new HoldCargo(intake, conveyor, feeder),
                new InstantCommand(shooter::stopShooter, shooter));
    }
}
