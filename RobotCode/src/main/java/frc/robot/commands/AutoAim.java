// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.Limelight;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class AutoAim extends SequentialCommandGroup {
  public AutoAim(DoubleSupplier turnToDegree, Limelight limelight, GTADrive drive) {
    SmartDashboard.putNumber("What to do", turnToDegree.getAsDouble());
    addCommands(new TurnToDegreeGyro(turnToDegree.getAsDouble(), drive));
  }
}
