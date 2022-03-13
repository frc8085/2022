// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

// Constants
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.AutoConstants.*;

// Inputs
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBaseSequence;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.EjectCargo;
import frc.robot.commands.HoldCargo;
import frc.robot.commands.LoadCargo;
import frc.robot.commands.Shoot;

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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
    // Add Auto Selection chooser to Dashboard
    protected SendableChooser<Command> m_autoSelection = new SendableChooser<>();

    // Drive train and driver controller
    private final XboxController m_driverController = new XboxController(kDriverControllerPort);
    private final GTADrive m_drive = new GTADrive(m_driverController);

    // Operator controller and subsystems
    private final XboxController m_operatorController = new XboxController(
            kOperatorControllerPort);
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final Conveyor m_conveyor = new Conveyor();
    private final IntakeCover m_intakeCover = new IntakeCover();
    private final Intake m_intake = new Intake();
    private final ClimberBrake m_climberBrake = new ClimberBrake();

    // The robot's subsystems and commands

    // TODO: Is there a better way to do this?
    // Because the Climber and Intake are using Joystick Axes, we're passing
    // the operator controllers to them instead of setting them here
    // shen using configureButtonBindings

    private final Climber m_climber = new Climber(m_operatorController);

    private final Command autoUpAgainstHub = new AutoBaseSequence(
            kTargetBumpedTBD, // shoot to desired target
            40, // drive
            kPickupCargo, // pick up new cargo
            -50, // drive back
            kTargetBumpedTBD, // shoot to desired target
            75, // turn
            70, // drive
            kPickupCargo, // pick up new cargo
            kStandStill, // 🚫 DON'T drive
            kShooterOff, // 🚫 don't shoot (set setpoint to 0)
            m_drive, m_intake, m_conveyor, m_feeder, m_shooter, m_intakeCover);

    private final Command autoSecondLocation = new AutoBaseSequence(
            kTargetBumpedTBD, // shoot to desired target
            40, // drive
            kPickupCargo, // pick up new cargo
            20, // drive forward
            kShooterOff, // 🚫 don't shoot (set setpoint to 0)
            kStandStill, // 🚫 don't turn
            -85, // drive backwards
            kDontPickupCargo, // 🚫 don't pick up new cargo
            kStandStill, // 🚫 don't drive
            kTargetBumpedTBD, // shoot to desired target
            m_drive, m_intake, m_conveyor, m_feeder, m_shooter, m_intakeCover);

    public RobotContainer() {
        configureButtonBindings();

        m_drive.setDefaultCommand(new Drive(m_drive));
        m_climber.setDefaultCommand(new Climb(m_climber, m_intakeCover, m_climberBrake));

        // Add commands to the autonomous command chooser
        m_autoSelection.setDefaultOption("Up Against Hub", autoUpAgainstHub);
        m_autoSelection.addOption("Across Line 2nd Ball High", autoSecondLocation);

        // Put the chooser on the dashboard

        Shuffleboard.getTab("Operator")
                .add("Auto routine", m_autoSelection)
                .withPosition(1, 0)
                .withSize(4, 1);

    }

    private void configureButtonBindings() {
        // Create some buttons
        final JoystickButton shooterOffButton = new JoystickButton(m_operatorController, Button.kX.value);
        final JoystickButton setTargetFar = new JoystickButton(m_operatorController, Button.kY.value);
        final JoystickButton setTargetTBD = new JoystickButton(m_operatorController, Button.kB.value);
        final JoystickButton setTargetNear = new JoystickButton(m_operatorController, Button.kA.value);
        final JoystickButton bumpTargetSpeeds = new JoystickButton(m_operatorController,
                Button.kRightBumper.value);

        // Create fake button to correspond to right trigger pressed
        final JoystickAxisButton shootButton = new JoystickAxisButton("Shoot",
                m_operatorController::getRightTriggerAxis, 0.5);

        // Create fake buttons to correspond to right joystick up / down
        final JoystickAxisButton cargoLoadControl = new JoystickAxisButton("Load",
                m_operatorController::getLeftY, 0.25);
        final JoystickAxisButton cargoEjectControl = new JoystickAxisButton("Eject",
                m_operatorController::getLeftY, -0.25);

        final JoystickButton unlockClimberButton = new JoystickButton(m_operatorController,
                Button.kBack.value);
        final JoystickButton lockClimberButton = new JoystickButton(m_operatorController,
                Button.kStart.value);

        // Create fake buttons from POV Dpad Values
        final DPadButton closeIntakeCoverButton = new DPadButton("Close intakeCover", m_operatorController,
                DPadButton.Value.kDPadUp);
        final DPadButton openIntakeCoverButton = new DPadButton("Open intakeCover", m_operatorController,
                DPadButton.Value.kDPadDown);

        /**
         * SET SHOOTING TARGET
         * Setting the shooting target will update the shooter motor setpoint
         */
        bumpTargetSpeeds.whenPressed(new InstantCommand(m_shooter::setBumped, m_shooter))
                .whenReleased(new InstantCommand(m_shooter::setNormal, m_shooter));

        setTargetFar.whenPressed(new InstantCommand(() -> m_shooter.setShootingMode(kTargetFar)));
        setTargetTBD.whenPressed(new InstantCommand(() -> m_shooter.setShootingMode(kTargetTBD)));
        setTargetNear.whenPressed(new InstantCommand(() -> m_shooter.setShootingMode(kTargetNear)));

        /**
         * SHOOT ACTION
         * Shooting will run the feeder, but only if the shooter is up to speed.
         * The speed depends on which shooting target mode we are in.
         * Releasing the shoot trigger will stop the feeder.
         */

        // TODO: Set time to automatically turn off the shooter motor
        // See Timer.getFPGATimestamp

        shootButton.whenPressed(new Shoot(m_intake, m_feeder, m_shooter, m_conveyor));

        shooterOffButton.whenPressed(new InstantCommand(m_shooter::stopShooter, m_shooter)
                .andThen(() -> m_shooter.setShootingMode(kShooterOff)));

        /**
         * LOAD CARGO
         * Open the intake cover when you run intake
         * When you release the cargo load button, hold the cargo by stopping all motors
         * Close the intake cover after 2 seconds without loading cargo
         */
        cargoLoadControl.whenHeld(
                new LoadCargo(m_intake, m_intakeCover, m_conveyor, m_feeder, m_shooter))
                .whenReleased(
                        new LoadCargo(m_intake, m_intakeCover, m_conveyor, m_feeder, m_shooter)
                                .withTimeout(kLoadLagSecs)
                                .andThen(new HoldCargo(m_intake, m_conveyor, m_feeder))
                                .andThen(new WaitCommand(10)
                                        .andThen(new InstantCommand(
                                                m_intakeCover::closeIntake,
                                                m_intakeCover))));

        /**
         * EJECT CARGO
         * When you release cargo eject, hold any remaining cargo by stopping all motors
         */
        cargoEjectControl.whenPressed(new EjectCargo(m_intake, m_conveyor, m_feeder, m_intakeCover))
                .whenReleased(new HoldCargo(m_intake, m_conveyor, m_feeder));

        /** INTAKE COVER MANUAL OPEN/CLOSE */
        openIntakeCoverButton.whenPressed(new InstantCommand(m_intakeCover::openIntake, m_intakeCover));

        closeIntakeCoverButton.whenPressed(
                new ConditionalCommand(
                        // If the climber is locked, you'e free to close the intake
                        new InstantCommand(m_intakeCover::closeIntake, m_intakeCover),
                        // If the climber is UNLOCKED, do not close intake
                        new InstantCommand(),
                        // Check if the climber locked
                        m_climber::isLocked));

        /** LOCK AND UNLOCK CLIMBER AND BRAKE */
        unlockClimberButton.whenPressed(new InstantCommand(m_climber::unlockClimber, m_climber)
                .andThen(new InstantCommand(m_climberBrake::unlockClimber, m_climberBrake)));
        lockClimberButton.whenPressed(
                new InstantCommand(m_climber::lockClimber, m_climber)
                        .andThen(new InstantCommand(m_climberBrake::lockClimber,
                                m_climberBrake)));

    }

    public Command getAutonomousCommand() {
        // Command to run in autonomous
        return m_autoSelection.getSelected();
    }
}
