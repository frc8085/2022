// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootLowNear extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public ShootLowNear(Intake intake, Feeder feeder, Shooter shooter, Conveyor conveyor) {
        shooter.setSetpoint(ShooterConstants.kShooterTargetRPM[OIConstants.kTargetLowNear]);
        addCommands(
                new InstantCommand(conveyor::runConveyor, conveyor)
                        .andThen(new WaitCommand(1))
                        .andThen(new Shoot(intake, feeder, shooter, conveyor))
                        .andThen(new HoldCargo(intake, conveyor, feeder)));

    }

}
