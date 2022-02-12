// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

// This has the intake + conveyor
import frc.robot.subsystems.Intake;

// This has the feeder
import frc.robot.subsystems.Shooter;

public class HoldCargo extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Intake m_intake_subsystem;
    private final Shooter m_shooter_subsystem;

    public HoldCargo(Intake intake_subsystem, Shooter shooter_subsystem) {
        m_intake_subsystem = intake_subsystem;
        m_shooter_subsystem = shooter_subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake_subsystem, shooter_subsystem);
    }

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
