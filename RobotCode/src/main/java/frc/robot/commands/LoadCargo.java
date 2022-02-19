// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hatch;
import frc.robot.subsystems.Intake;

// Run feeder and conveyor in the same direction at a set speed.
// Make sure that the feeder is not running
public class LoadCargo extends SequentialCommandGroup {
    public LoadCargo(Intake intake, Hatch hatch, Conveyor conveyor, Feeder feeder) {
        addCommands(
                new InstantCommand(hatch::openIntake, hatch),
                new InstantCommand(feeder::stopFeeder, feeder),
                new InstantCommand(intake::runIntake, intake),
                new InstantCommand(conveyor::runConveyor, conveyor));
    }

}
