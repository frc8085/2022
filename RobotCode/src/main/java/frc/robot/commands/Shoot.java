// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/**
 * Run the feeder and conveyor to shoot, but only if the shooter is up to speed
 */

public class Shoot extends SequentialCommandGroup {
  public Shoot(Intake intake, Feeder feeder, Shooter shooter, Conveyor conveyor) {
    addCommands(
        new ConditionalCommand(
            // If shooter is up to speed, run the feeder & conveyor
            new InstantCommand(feeder::runFeeder, feeder).withTimeout(ShooterConstants.kShootBurstTime)
                .alongWith(new InstantCommand(conveyor::runConveyor, conveyor))
                .withTimeout(ShooterConstants.kShootBurstTime)
                // Stop the shooting routine after N seconds
                .andThen(new InstantCommand(feeder::stopFeeder, feeder)
                    .alongWith(new InstantCommand(conveyor::stopConveyor, conveyor))),

            // If shooter is NOT up to speed, do nothing
            new InstantCommand(),
            // Check if shooter is up to speed
            shooter::atSetpoint));
  }
}
