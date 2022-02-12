// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
// This has the intake + conveyor
import frc.robot.subsystems.Intake;

// This has the feeder
import frc.robot.subsystems.Shooter;

// Run feeder and conveyor in the same direction at a set speed.
// Make sure that the shooter is at 0

public class HoldCargo extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public HoldCargo(Intake intake, Feeder feeder, Shooter shooter) {
        // addCommands(
        // new OpenClaw(claw),
        // parallel(new SetWristSetpoint(0, wrist), new SetElevatorSetpoint(0,
        // elevator)));
    }

}
