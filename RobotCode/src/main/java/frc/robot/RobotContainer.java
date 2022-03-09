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

// Displays
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

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

        /**
         * Shooting Mode definition
         * shootMode is an integer corresponding to the different types of targets
         * shootingMode translates the integer into a string so we can display it
         * in the Shuffleboard entry shotingModeDisplay
         */
        private NetworkTableEntry shootingModeDisplay;
        private int shootMode = kShooterOff;
        private boolean shootLow = false;

        private static final Map<Integer, String> shootingMode = new HashMap<Integer, String>() {
                {
                        put(kShooterOff, "Shooting mode not selected");

                        put(kTargetHighNear, "HIGH ▔ Near");
                        put(kTargetHighFar, "HIGH ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔ Far");
                        put(kTargetHighAngled, "HIGH ▔▔/angled/▔▔");

                        put(kTargetLowNear, "LOW _ Near");
                        put(kTargetLowFar, "LOW ____________________ Far");
                        put(kTargetLowAngled, "LOW __/angled/__");
                }
        };

        private final Command autoUpAgainstHub = new AutoBaseSequence(
                        kTargetHighNear, // shoot to desired target
                        64, // drive
                        kPickupCargo, // pick up new cargo
                        -64, // drive back
                        kTargetHighNear, // shoot to desired target
                        90, // turn
                        72, // drive
                        kPickupCargo, // pick up new cargo
                        kStandStill, // 🚫 DON'T drive
                        m_drive, m_intake, m_conveyor, m_feeder, m_shooter, m_intakeCover);

        private final Command autoTwoBallHigh = new AutoBaseSequence(
                        kTargetHighFar, // shoot to desired target
                        24, // drive
                        kDontPickupCargo, // 🚫 don't pick up new cargo
                        kStandStill, // 🚫 don't drive
                        kShooterOff, // 🚫 don't shoot (set setpoint to 0)
                        kStandStill, // 🚫 don't turn
                        kStandStill, // 🚫 don't drive
                        kDontPickupCargo, // 🚫 don't pick up new cargo
                        kStandStill, // 🚫 don't drive
                        m_drive, m_intake, m_conveyor, m_feeder, m_shooter, m_intakeCover);

        public RobotContainer() {
                configureButtonBindings();

                m_drive.setDefaultCommand(new Drive(m_drive));
                m_climber.setDefaultCommand(new Climb(m_climber, m_intakeCover, m_climberBrake));

                // Add commands to the autonomous command chooser
                m_autoSelection.setDefaultOption("Up Against Hub", autoUpAgainstHub);
                m_autoSelection.addOption("Across Line 2 Ball High", autoTwoBallHigh);

                // Put the chooser on the dashboard
                SmartDashboard.putData(m_autoSelection);

                // TODO: Move this to the Shooter submodue
                shootingModeDisplay = Shuffleboard.getTab("Shooter")
                                .add("Shooting Mode", shootingMode.get(shootMode))
                                .withPosition(0, 0)
                                .withSize(2, 1)
                                .getEntry();

        }

        private void configureButtonBindings() {
                // Create some buttons
                final JoystickButton shooterOffButton = new JoystickButton(m_operatorController, Button.kX.value);
                final JoystickButton setTargetFar = new JoystickButton(m_operatorController, Button.kY.value);
                final JoystickButton setTargetAngled = new JoystickButton(m_operatorController, Button.kB.value);
                final JoystickButton setTargetNear = new JoystickButton(m_operatorController, Button.kA.value);
                final JoystickButton setLowTarget = new JoystickButton(m_operatorController, Button.kRightBumper.value);

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
                setLowTarget.whenPressed(
                                new InstantCommand(() -> setLow(shootMode)))
                                .whenReleased(new InstantCommand(() -> setHigh(shootMode)));

                setTargetFar.whenPressed(
                                new ConditionalCommand(
                                                new InstantCommand(() -> setShootingMode(kTargetLowFar)),
                                                new InstantCommand(() -> setShootingMode(kTargetHighFar)),
                                                () -> shootLow));

                setTargetAngled.whenPressed(
                                new ConditionalCommand(
                                                new InstantCommand(() -> setShootingMode(kTargetLowAngled)),
                                                new InstantCommand(() -> setShootingMode(kTargetHighAngled)),
                                                () -> shootLow));

                setTargetNear.whenPressed(
                                new ConditionalCommand(
                                                new InstantCommand(() -> setShootingMode(kTargetLowNear)),
                                                new InstantCommand(() -> setShootingMode(kTargetHighNear)),
                                                () -> shootLow));

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
                                .andThen(() -> setShootingMode(kShooterOff)));

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
                closeIntakeCoverButton.whenPressed(new InstantCommand(m_intakeCover::closeIntake, m_intakeCover));

                /** LOCK AND UNLOCK CLIMBER AND BRAKE */
                unlockClimberButton.whenPressed(new InstantCommand(m_climber::unlockClimber, m_climber)
                                .andThen(new InstantCommand(m_climberBrake::unlockClimber, m_climberBrake)));
                lockClimberButton.whenPressed(
                                new InstantCommand(m_climber::lockClimber, m_climber)
                                                .andThen(new InstantCommand(m_climberBrake::lockClimber,
                                                                m_climberBrake)));

        }

        private void setShootingMode(int mode) {
                shootMode = mode;
                shootingModeDisplay.setString(shootingMode.get(shootMode));
                m_shooter.setSetpoint(kShooterTargetRPM[mode]);
        }

        private void setLow(int mode) {
                shootLow = true;

                switch (mode) {
                        case kTargetHighNear:
                                shootMode = kTargetLowNear;
                                break;
                        case kTargetHighFar:
                                shootMode = kTargetLowFar;
                                break;
                        case kTargetHighAngled:
                                shootMode = kTargetLowAngled;
                                break;
                }

                setShootingMode(shootMode);
        }

        private void setHigh(int mode) {
                shootLow = false;

                switch (mode) {
                        case kTargetLowNear:
                                shootMode = kTargetHighNear;
                                break;
                        case kTargetLowFar:
                                shootMode = kTargetHighFar;
                                break;
                        case kTargetLowAngled:
                                shootMode = kTargetHighAngled;
                                break;
                }

                setShootingMode(shootMode);
        }

        public Command getAutonomousCommand() {
                // Command to run in autonomous
                return m_autoSelection.getSelected();
        }
}
