// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootLowNear extends SequentialCommandGroup {
    public ShootLowNear(Intake intake, Feeder feeder, Shooter shooter, Conveyor conveyor) {
        addCommands(new InstantCommand(() -> shooter.setSetpoint(kShooterTargetRPM[kTargetLowNear]))
                .andThen(new InstantCommand(conveyor::runConveyor, conveyor)
                        .andThen(new WaitCommand(1.5))
                        .andThen(new Shoot(intake, feeder, shooter, conveyor))
                        .andThen(new HoldCargo(intake, conveyor, feeder)
                                .andThen(new InstantCommand(shooter::stopShooter, shooter)))));

    }

}
