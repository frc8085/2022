// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Constants
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.AutoConstants.*;

// Inputs
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.commands.AutoRoutines.FourShot_PickupShootShoot;
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
import frc.robot.utilities.LimelightConfiguration.LedMode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.ClimberBrake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
        // Add Auto Selection chooser to Dashboard
        protected SendableChooser<Command> autoSelection = new SendableChooser<>();

        // Operator controller and subsystems
        CommandXboxController m_operatorController = new CommandXboxController(kOperatorControllerPort);

        XboxController m_drivercontroller = new XboxController(kDriverControllerPort);

        private final Shooter shooter = new Shooter();
        private final Feeder feeder = new Feeder();
        private final Conveyor conveyor = new Conveyor();
        private final IntakeCover intakeCover = new IntakeCover();
        private final Intake intake = new Intake();
        private final ClimberBrake climberBrake = new ClimberBrake();
        private final Climber climber = new Climber(m_operatorController);

        // Drive train and driver controller
        private final XboxController driverController = new XboxController(kDriverControllerPort);

        // private final GTADrive drive = new GTADrive(driverController, climber);
        private final GTADrive drive = new GTADrive(driverController, climber);

        private final Limelight limelight = new Limelight();

        /** LimeLight AUTOS */
        private final Command fiveShot = new FiveShot_ParallelToWall(
                        limelight, drive, intake, conveyor, feeder, shooter, intakeCover);
        private final Command threeShot = new ThreeShot_ParallelToWall(
                        limelight, drive, intake, conveyor, feeder, shooter, intakeCover);
        private final Command twoShot = new TwoShot_PickupShootShoot(
                        limelight, drive, intake, conveyor, feeder, shooter, intakeCover);
        private final Command twoShotB = new TwoShot_ShootPickupShoot(
                        limelight, drive, intake, conveyor, feeder, shooter, intakeCover);
        private final Command fourShot = new FourShot_PickupShootShoot(
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
                DriverStation.silenceJoystickConnectionWarning(true);

                configureButtonBindings();

                // Make sure the limelight's LED is off when we turn on
                turnOffLimelightLED();

                drive.setDefaultCommand(new Drive(drive));
                climber.setDefaultCommand(new Climb(climber, intakeCover, climberBrake));
                // Add commands to the autonomous command chooser

                autoSelection.setDefaultOption("1 Shot - By Hangar No Cargo Pickup", fiveShot);
                autoSelection.addOption("3 Shot - Parallel to wall", threeShot);
                autoSelection.addOption("2 Shot - 3-Shot Position but no 3rd ball pickup", fourShot);
                autoSelection.addOption("2 Shot Defensive - Pickup then shoot twice", twoShot);
                autoSelection.addOption("MANUAL - 2 Shot - Shoot first, pickup, shoot", twoShotB);
                // autoSelection.addOption("MANUAL - Up Against Hub", autoUpAgainstHub);
                // autoSelection.addOption("MANUAL - Across Line 2nd Ball High",
                // autoSecondLocation);

                // Put the chooser on the dashboard
                SmartDashboard.putData("Auto Routine", autoSelection);
        }

        private void configureButtonBindings() {

                /** MANUAL OPERATION */
                final Trigger shooterOffButton = m_operatorController.x();
                final Trigger setTargetFar = m_operatorController.y();
                final Trigger setTargetTBD = m_operatorController.b();
                final Trigger setTargetNear = m_operatorController.a();

                final Trigger shootButton = m_operatorController.rightTrigger();

                // FOR OYSTER FESTIVAL
                final Trigger cargoLoadControl = m_operatorController.povDown();
                final Trigger cargoEjectControl = m_operatorController.povUp();

                final Trigger lockClimberButton = m_operatorController.start();

                final Trigger openIntakeCoverButton = m_operatorController.povLeft();
                final Trigger closeIntakeCoverButton = m_operatorController.povRight();

                // final JoystickButton unlockClimberButton = new
                // JoystickButton(operatorController,
                // Button.kBack.value);

                /**
                 * SET SHOOTING TARGET
                 * Setting the shooting target will update the shooter motor setpoint
                 */
                // bumpTargetSpeeds.whenPressed(new InstantCommand(shooter::setBumped, shooter))
                // .whenReleased(new InstantCommand(shooter::setNormal, shooter));

                setTargetFar.onTrue(new InstantCommand(() -> shooter.setShootingMode(kTargetFar)));
                setTargetTBD.onTrue(new InstantCommand(() -> shooter.setShootingMode(kTargetTBD)));
                setTargetNear.onTrue(new InstantCommand(() -> shooter.setShootingMode(kTargetNear)));

                /**
                 * SHOOT ACTION
                 * Shooting will run the feeder, but only if the shooter is up to speed.
                 * The speed depends on which shooting target mode we are in.
                 * Releasing the shoot trigger will stop the feeder.
                 */

                // TODO: Set time to automatically turn off the shooter motor
                // See Timer.getFPGATimestamp

                shootButton.onTrue(new Shoot(intake, feeder, shooter, conveyor));

                shooterOffButton.onTrue(new InstantCommand(shooter::stopShooter, shooter)
                                .andThen(() -> shooter.setShootingMode(kShooterOff)));

                /**
                 * LOAD CARGO
                 * Open the intake cover when you run intake
                 * When you release the cargo load button, hold the cargo by stopping all motors
                 * Close the intake cover after 2 seconds without loading cargo
                 */
                cargoLoadControl.whileTrue(
                                new LoadCargo(intake, intakeCover, conveyor, feeder, shooter))
                                .onFalse(
                                                new LoadCargo(intake, intakeCover, conveyor, feeder, shooter)
                                                                .withTimeout(kLoadLagSecs)
                                                                .andThen(new HoldCargo(intake, conveyor, feeder)));

                /**
                 * EJECT CARGO
                 * When you release cargo eject, hold any remaining cargo by stopping all motors
                 */
                cargoEjectControl.whileTrue(new EjectCargo(intake, conveyor, feeder, intakeCover))
                                .onFalse(new HoldCargo(intake, conveyor, feeder));

                /** INTAKE COVER MANUAL OPEN/CLOSE */
                openIntakeCoverButton.onTrue(new InstantCommand(intakeCover::openIntake, intakeCover));

                closeIntakeCoverButton.onTrue(
                                new ConditionalCommand(
                                                // If the climber is locked, you'e free to close the intake
                                                new InstantCommand(intakeCover::closeIntake, intakeCover),
                                                // // If the climber is UNLOCKED, do not close intake
                                                new InstantCommand(),
                                                // // Check if the climber locked
                                                climber::isLocked));

                /** LOCK AND UNLOCK CLIMBER AND BRAKE */
                // unlockClimberButton.whileTrue(new SequentialCommandGroup(
                // new InstantCommand(climber::unlockClimber, climber),
                // new InstantCommand(climberBrake::unlockClimber, climberBrake),
                // // Turn off LL when climbing
                // new InstantCommand(() -> limelight.setLEDMode(LedMode.kforceOff))));

                lockClimberButton.onTrue(new SequentialCommandGroup(
                                new InstantCommand(climber::lockClimber, climber),
                                new InstantCommand(climberBrake::lockClimber, climberBrake),
                                // Turn on LL when not climbing
                                new InstantCommand(() -> limelight.setLEDMode(LedMode.kforceOn))));

        }

        public void turnOffLimelightLED() {
                limelight.setLEDMode(LedMode.kforceOff);
        }

        public void turnOnLimelightLED() {
                limelight.setLEDMode(LedMode.kforceOn);
        }

        public Command getAutonomousCommand() {
                // Command to run in autonomous
                return autoSelection.getSelected();
        }
}
