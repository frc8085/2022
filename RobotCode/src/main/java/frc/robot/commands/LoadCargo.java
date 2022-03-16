// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// Run feeder and conveyor in the same direction at a set speed.
// Make sure that the feeder is not running

public class LoadCargo extends SequentialCommandGroup {
    public LoadCargo(Intake intake, IntakeCover intakeCover, Conveyor conveyor, Feeder feeder, Shooter shooter) {
        addCommands(new ConditionalCommand(
                // Don't open the intake cover if it's already down/open
                new InstantCommand(),
                // Open the intake cover if it's not down/open
                new InstantCommand(intakeCover::openIntake, intakeCover).andThen(new WaitCommand(1)),
                // Check if the intake cover is down/open
                intakeCover::isIntakeCoverDown),
                // Run the intake and conveyor
                parallel(new InstantCommand(intake::runIntake, intake),
                        new InstantCommand(conveyor::runConveyor, conveyor)));

    }

}
