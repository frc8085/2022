// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Shooter;
import static frc.robot.utilities.GetSetpointFromDistance.setpointFromDistance;

/**
 * Automatically determine the correct shooting setpoint, get to it, and shoot.
 * 1. Use Limelight to turn to the target
 * 2. Use Limelight to estimate the area of the target
 * 3. Estimate our distance to the target based on the area on the Limelight
 * image
 * 4. Calculate the setpoint based on estimated distance
 * 5. Turn to the target, get up to speed and shoot
 */

public class AutoSetpointShot extends SequentialCommandGroup {
    public AutoSetpointShot(
            GTADrive drive,
            Limelight limelight,
            Intake intake,
            Feeder feeder,
            Shooter shooter,
            Conveyor conveyor) {

        double distanceToTargetInches = limelight.getDistanceToTarget();
        double autoSetpoint = setpointFromDistance(distanceToTargetInches);

        addCommands(new ConditionalCommand(
                sequence(new AutoAimWithLimelight(drive, limelight),
                        new InstantCommand(() -> shooter.setSetpoint(autoSetpoint)),
                        new InstantCommand(conveyor::runConveyor, conveyor),
                        new WaitCommand(autoSetpoint),
                        new Shoot(intake, feeder, shooter, conveyor),
                        new HoldCargo(intake, conveyor, feeder),
                        new InstantCommand(shooter::stopShooter, shooter)),
                // Do nothing if we don't actually see a target
                new InstantCommand(),
                limelight::getIsTargetFound));
    }
}
