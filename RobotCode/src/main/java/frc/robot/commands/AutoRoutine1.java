// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Shooter;

/** The main autonomous command */
public class AutoRoutine1 extends SequentialCommandGroup {
  /** Create a new autonomous command. */
  public AutoRoutine1(
      GTADrive drive,
      Shooter shooter,
      Feeder feeder,
      Conveyor conveyor,
      IntakeCover intakeCover,
      Intake intake) {
    addCommands(
        new ShootLowNear(intake, feeder, shooter, conveyor)
            .andThen(new WaitCommand(1))
            .andThen(new TurnToDegreeGyro(drive.getHeading(), 90, drive)));
  }
}
