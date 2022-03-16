// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/** The shooting routine if we shoot before picking up second cargo */
public class Auto1Shot extends SequentialCommandGroup {
    /**
     * If we just plan to make one shot, shoot right away
     */
    public Auto1Shot(
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter) {
        addCommands(new ShootAuto(OIConstants.kTargetBumpedNear, intake, conveyor, feeder, shooter));
    }
}