// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HoldCargo;
import frc.robot.commands.LoadCargo;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Shooter;

// Load cargo in Auto. This makes sure it runs for a min. amount of seconds
public class LoadCargoAuto extends SequentialCommandGroup {
    private double loadCargoSecs = 2;

    public LoadCargoAuto(
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter,
            IntakeCover intakeCover) {

        addCommands(
                new LoadCargo(intake, intakeCover, conveyor, feeder, shooter),
                new WaitCommand(loadCargoSecs),
                new HoldCargo(intake, conveyor, feeder));
    }
}
