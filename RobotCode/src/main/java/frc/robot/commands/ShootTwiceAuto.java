// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// Unlike the teleop Shoot command, this one is not initiated/controlled
// by the operator. We need to automatically run the shooter to speed
// and add an extra conveyor run to make sure the cargo is loaded
public class ShootTwiceAuto extends SequentialCommandGroup {
    private double shootDurationSecs = 0.6;

    public ShootTwiceAuto(DoubleSupplier setpointGetter, Intake intake, Feeder feeder, Shooter shooter,
            Conveyor conveyor) {
        double setpoint = setpointGetter.getAsDouble();

        addCommands(
                new InstantCommand(() -> shooter.setSetpoint(setpoint)),
                new InstantCommand(conveyor::runConveyor, conveyor),
                new WaitUntilCommand(shooter::atSetpoint),
                new Shoot(intake, feeder, shooter, conveyor),
                new WaitUntilCommand(shooter::atSetpoint),
                new Shoot(intake, feeder, shooter, conveyor),
                new HoldCargo(intake, conveyor, feeder),
                new WaitCommand(shootDurationSecs * 2),
                new InstantCommand(shooter::stopShooter, shooter));
    }
}
