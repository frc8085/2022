// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;

// Run feeder and conveyor in the same direction at a set speed.
// Make sure that the feeder is not running
public class EjectCargo extends SequentialCommandGroup {
  public EjectCargo(Intake intake, Conveyor conveyor, Feeder feeder, IntakeCover intakeCover) {
    addCommands(
        // Briefly run feeder before ejecting
        new InstantCommand(feeder::runFeeder, feeder).withTimeout(1)

            // Check if intake cover is down/open
            .andThen(new ConditionalCommand(
                // if intake cover is down, run feeder, conveyor, and intake motors in reverse
                new InstantCommand(feeder::reverseFeeder, feeder)
                    .andThen(new InstantCommand(conveyor::reverseConveyor, conveyor))
                    .andThen(new InstantCommand(intake::reverseIntake, intake)),

                // if intake cover is up, run feeder and conveyor in reverse
                new InstantCommand(feeder::reverseFeeder, feeder)
                    .andThen(new InstantCommand(conveyor::reverseConveyor, conveyor)),

                // check condition if intake cover is down
                intakeCover::isIntakeCoverDown)));

  }

}