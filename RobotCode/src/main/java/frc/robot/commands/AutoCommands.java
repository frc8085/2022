// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Constants
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.AutoConstants.*;

// Subsystems
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DPadButton;
import frc.robot.utilities.JoystickAxisButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.ClimberBrake;

/** An example command that uses an example subsystem. */
public class AutoCommands extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private GTADrive m_drive;
    private Intake m_intake;
    private Conveyor m_conveyor;
    private Feeder m_feeder;
    private Shooter m_shooter;
    private IntakeCover m_intakeCover;

    /**
     * Creates a new AutoCommands.
     */
    public AutoCommands(
            GTADrive drive,
            Intake intake,
            Conveyor conveyor,
            Feeder feeder,
            Shooter shooter,
            IntakeCover intakeCover) {

        m_drive = drive;
        m_intake = intake;
        m_conveyor = conveyor;
        m_feeder = feeder;
        m_shooter = shooter;
        m_intakeCover = intakeCover;

        addRequirements(drive, intake, conveyor, feeder, shooter, intakeCover);
    }

    public final Command autoUpAgainstHub = new AutoBaseSequence(
            kTargetNear, // shoot to desired target
            64, // drive
            kPickupCargo, // pick up new cargo
            -64, // drive back
            kTargetNear, // shoot to desired target
            90, // turn
            72, // drive
            kPickupCargo, // pick up new cargo
            kStandStill, // 🚫 DON'T drive
            m_drive, m_intake, m_conveyor, m_feeder, m_shooter, m_intakeCover);

    public final Command autoTwoBallHigh = new AutoBaseSequence(
            kTargetFar, // shoot to desired target
            24, // drive
            kDontPickupCargo, // 🚫 don't pick up new cargo
            kStandStill, // 🚫 don't drive
            kShooterOff, // 🚫 don't shoot (set setpoint to 0)
            kStandStill, // 🚫 don't turn
            kStandStill, // 🚫 don't drive
            kDontPickupCargo, // 🚫 don't pick up new cargo
            kStandStill, // 🚫 don't drive
            m_drive, m_intake, m_conveyor, m_feeder, m_shooter, m_intakeCover);

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
