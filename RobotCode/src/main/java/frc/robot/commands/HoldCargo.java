// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

// Run feeder and conveyor in the same direction at a set speed.
// Make sure that the feeder is not running
public class HoldCargo extends SequentialCommandGroup {
    public HoldCargo(Intake intake, Conveyor conveyor, Feeder feeder) {
        addCommands(new InstantCommand(intake::stopIntake, intake)
                .alongWith(new InstantCommand(feeder::stopFeeder, feeder))
                .alongWith(new InstantCommand(conveyor::stopConveyor, conveyor)));
    }
}
