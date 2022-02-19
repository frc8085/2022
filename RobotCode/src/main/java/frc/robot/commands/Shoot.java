// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {
    public Shoot(Intake intake, Feeder feeder, Shooter shooter) {
        /**
         * Run the feeder to shoot, but only
         * if the shooter is at set point
         */
        new ConditionalCommand(
                new InstantCommand(feeder::runFeeder, feeder),
                new InstantCommand(),
                shooter::atSetpoint);
    }
}
