// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;

import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// Unlike the teleop Shoot command, this one is not initiated/controlled
// by the operator. We need to automatically run the shooter to speed
// and add an extra conveyor run to make sure the cargo is loaded
public class ShootWhileLoadingAuto extends SequentialCommandGroup {
    private final double m_intakeDurationSeconds = 2;

    public ShootWhileLoadingAuto(
            int shootingMode,
            Intake intake,
            IntakeCover intakeCover,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter) {

        Command shootCargo = sequence(
                // Get the shooter to the right setpoint
                new InstantCommand(() -> shooter.setSetpoint(kShooterTargetRPM[shootingMode])),
                new WaitUntilCommand(shooter::atSetpoint),
                // Shoot one cargo. DONT't stop the shooter, intake, or conveyor
                new Shoot(intake, feeder, shooter, conveyor));

        Command pickupCargo = sequence(
                new WaitCommand(m_intakeDurationSeconds)
                        .deadlineWith(new LoadCargo(intake, intakeCover, conveyor, feeder, shooter)),
                new HoldCargo(intake, conveyor, feeder));

        addCommands(shootCargo, pickupCargo);
    }
}
