// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Constants
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.OIConstants.*;
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
import frc.robot.commands.AutoAimWithLimelight;
import frc.robot.commands.AutoBaseSequence;
import frc.robot.commands.AutoSetpointShot;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.EjectCargo;
import frc.robot.commands.HoldCargo;
import frc.robot.commands.LoadCargo;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.AutoRoutines.FiveShot_ParallelToWall;
import frc.robot.commands.AutoRoutines.ThreeShot_ParallelToWall;
import frc.robot.commands.AutoRoutines.TwoShot_PickupShootShoot;
import frc.robot.commands.AutoRoutines.TwoShot_ShootPickupShoot;
// Subsystems
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GTADrive;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DPadButton;
import frc.robot.utilities.JoystickAxisButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.ClimberBrake;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
        // Add Auto Selection chooser to Dashboard
        protected SendableChooser<Command> autoSelection = new SendableChooser<>();

        // Drive train and driver controller
        private final XboxController driverController = new XboxController(kDriverControllerPort);
        private final GTADrive drive = new GTADrive(driverController);
        private final Limelight limelight = new Limelight();
        // Operator controller and subsystems
        private final XboxController operatorController = new XboxController(
                        kOperatorControllerPort);
        private final Shooter shooter = new Shooter();
        private final Feeder feeder = new Feeder();
        private final Conveyor conveyor = new Conveyor();
        private final IntakeCover intakeCover = new IntakeCover();
        private final Intake intake = new Intake();
        private final ClimberBrake climberBrake = new ClimberBrake();

        // The robot's subsystems and commands

        // TODO: Is there a better way to do this?
        // Because the Climber and Intake are using Joystick Axes, we're passing
        // the operator controllers to them instead of setting them here
        // shen using configureButtonBindings

        private final Climber climber = new Climber(operatorController);

        /** LimeLight AUTOS */
        private final Command fiveShot = new FiveShot_ParallelToWall(
                        limelight, drive, intake, conveyor, feeder, shooter, intakeCover);
        private final Command threeShot = new ThreeShot_ParallelToWall(
                        limelight, drive, intake, conveyor, feeder, shooter, intakeCover);
        private final Command twoShot = new TwoShot_ShootPickupShoot(
                        limelight, drive, intake, conveyor, feeder, shooter, intakeCover);
        private final Command twoShotB = new TwoShot_PickupShootShoot(
                        limelight, drive, intake, conveyor, feeder, shooter, intakeCover);

        /** MANUAL AUTOS */
        private final Command autoUpAgainstHub = new AutoBaseSequence(
                        kShooterOff, // shoot to desired target
                        40, // drive
                        kPickupCargo, // pick up new cargo
                        kStandStill, // drive back
                        kTargetFar, // shoot to desired target
                        75, // turn
                        70, // drive
                        kPickupCargo, // pick up new cargo
                        kStandStill, // 🚫 DON'T drive
                        kShooterOff, // 🚫 don't shoot (set setpoint to 0)
                        drive, intake, conveyor, feeder, shooter, intakeCover);

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
                        drive, intake, conveyor, feeder, shooter, intakeCover);

        public RobotContainer() {
                configureButtonBindings();

                drive.setDefaultCommand(new Drive(drive));
                climber.setDefaultCommand(new Climb(climber, intakeCover, climberBrake));
                // Add commands to the autonomous command chooser

                autoSelection.setDefaultOption("LIMELIGHT - 5 Shot - Parallel to wall", fiveShot);
                autoSelection.addOption("LIMELIGHT - 3 Shot - Parallel to wall", threeShot);
                autoSelection.addOption("MANUAL - 2 Shot - Shoot, Pickup, Shoot", twoShot);
                autoSelection.addOption("MANUAL - 2 Shot - Pickup, Shoot both", twoShotB);
                autoSelection.addOption("MANUAL - Up Against Hub", autoUpAgainstHub);
                autoSelection.addOption("MANUAL - Across Line 2nd Ball High", autoSecondLocation);

                // Put the chooser on the dashboard
                SmartDashboard.putData("Auto Routine", autoSelection);
        }

        private void configureButtonBindings() {
                /** AUTOMATIC OPERATION (using limelight) */
                // Driver can auto aim to target

                final JoystickButton autoAimButton = new JoystickButton(driverController, Button.kRightBumper.value);
                autoAimButton.whenPressed(new AutoAimWithLimelight(drive, limelight));

                final JoystickButton autoShootDriverButton = new JoystickButton(driverController, Button.kB.value);
                autoShootDriverButton.whenPressed(
                                new AutoSetpointShot(drive, limelight, intake, feeder, shooter, conveyor));

                // Operator can auto shoot. This also auto aims.
                final JoystickButton autoShootButton = new JoystickButton(operatorController, Button.kLeftBumper.value);
                autoShootButton.whenPressed(
                                new AutoSetpointShot(drive, limelight, intake, feeder, shooter, conveyor));

                /** MANUAL OPERATION */
                final JoystickButton shooterOffButton = new JoystickButton(operatorController, Button.kX.value);
                final JoystickButton setTargetFar = new JoystickButton(operatorController, Button.kY.value);
                final JoystickButton setTargetTBD = new JoystickButton(operatorController, Button.kB.value);
                final JoystickButton setTargetNear = new JoystickButton(operatorController, Button.kA.value);
                final JoystickButton bumpTargetSpeeds = new JoystickButton(operatorController,
                                Button.kRightBumper.value);

                // Create fake button to correspond to right trigger pressed
                final JoystickAxisButton shootButton = new JoystickAxisButton("Shoot",
                                operatorController::getRightTriggerAxis, 0.5);

                // Create fake buttons to correspond to right joystick up / down
                final JoystickAxisButton cargoLoadControl = new JoystickAxisButton("Load",
                                operatorController::getLeftY, 0.25);
                final JoystickAxisButton cargoEjectControl = new JoystickAxisButton("Eject",
                                operatorController::getLeftY, -0.25);

                final JoystickButton unlockClimberButton = new JoystickButton(operatorController,
                                Button.kBack.value);
                final JoystickButton lockClimberButton = new JoystickButton(operatorController,
                                Button.kStart.value);

                // Create fake buttons from POV Dpad Values
                final DPadButton closeIntakeCoverButton = new DPadButton("Close intakeCover", operatorController,
                                DPadButton.Value.kDPadUp);
                final DPadButton openIntakeCoverButton = new DPadButton("Open intakeCover", operatorController,
                                DPadButton.Value.kDPadDown);

                /**
                 * SET SHOOTING TARGET
                 * Setting the shooting target will update the shooter motor setpoint
                 */
                bumpTargetSpeeds.whenPressed(new InstantCommand(shooter::setBumped, shooter))
                                .whenReleased(new InstantCommand(shooter::setNormal, shooter));

                setTargetFar.whenPressed(new InstantCommand(() -> shooter.setShootingMode(kTargetFar)));
                setTargetTBD.whenPressed(new InstantCommand(() -> shooter.setShootingMode(kTargetTBD)));
                setTargetNear.whenPressed(new InstantCommand(() -> shooter.setShootingMode(kTargetNear)));

                /**
                 * SHOOT ACTION
                 * Shooting will run the feeder, but only if the shooter is up to speed.
                 * The speed depends on which shooting target mode we are in.
                 * Releasing the shoot trigger will stop the feeder.
                 */

                // TODO: Set time to automatically turn off the shooter motor
                // See Timer.getFPGATimestamp

                shootButton.whenPressed(new Shoot(intake, feeder, shooter, conveyor));

                shooterOffButton.whenPressed(new InstantCommand(shooter::stopShooter, shooter)
                                .andThen(() -> shooter.setShootingMode(kShooterOff)));

                /**
                 * LOAD CARGO
                 * Open the intake cover when you run intake
                 * When you release the cargo load button, hold the cargo by stopping all motors
                 * Close the intake cover after 2 seconds without loading cargo
                 */
                cargoLoadControl.whenHeld(
                                new LoadCargo(intake, intakeCover, conveyor, feeder, shooter))
                                .whenReleased(
                                                new LoadCargo(intake, intakeCover, conveyor, feeder, shooter)
                                                                .withTimeout(kLoadLagSecs)
                                                                .andThen(new HoldCargo(intake, conveyor, feeder))
                                                                .andThen(new WaitCommand(10)
                                                                                .andThen(new InstantCommand(
                                                                                                intakeCover::closeIntake,
                                                                                                intakeCover))));

                /**
                 * EJECT CARGO
                 * When you release cargo eject, hold any remaining cargo by stopping all motors
                 */
                cargoEjectControl.whenPressed(new EjectCargo(intake, conveyor, feeder, intakeCover))
                                .whenReleased(new HoldCargo(intake, conveyor, feeder));

                /** INTAKE COVER MANUAL OPEN/CLOSE */
                openIntakeCoverButton.whenPressed(new InstantCommand(intakeCover::openIntake, intakeCover));

                closeIntakeCoverButton.whenPressed(
                                new ConditionalCommand(
                                                // If the climber is locked, you'e free to close the intake
                                                new InstantCommand(intakeCover::closeIntake, intakeCover),
                                                // If the climber is UNLOCKED, do not close intake
                                                new InstantCommand(),
                                                // Check if the climber locked
                                                climber::isLocked));

                /** LOCK AND UNLOCK CLIMBER AND BRAKE */
                unlockClimberButton.whenPressed(new InstantCommand(climber::unlockClimber, climber)
                                .andThen(new InstantCommand(climberBrake::unlockClimber, climberBrake)));
                lockClimberButton.whenPressed(
                                new InstantCommand(climber::lockClimber, climber)
                                                .andThen(new InstantCommand(climberBrake::lockClimber,
                                                                climberBrake)));

        }

        public Command getAutonomousCommand() {
                // Command to run in autonomous
                return autoSelection.getSelected();
        }
}
