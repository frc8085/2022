// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GTADrive;

/** The main autonomous command */
public class AutoRoutine1 extends SequentialCommandGroup {
  /** Create a new autonomous command. */
  public AutoRoutine1(GTADrive drive) {
    addCommands(
        new DriveStraight(0.1, drive));
  }
}
