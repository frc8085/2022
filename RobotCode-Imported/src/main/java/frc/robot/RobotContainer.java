// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Constants
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.wpilibj.Joystick;
// Inputs
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.subsystems.TankDrive;
import frc.robot.utilities.JoystickAxisButton;
import frc.robot.utilities.LimelightConfiguration.LedMode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.ClimberBrake;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    // Add Auto Selection chooser to Dashboard
    protected SendableChooser<Command> autoSelection = new SendableChooser<>();

    // Operator controller and subsystems
    private final CommandXboxController operatorController = new CommandXboxController(
            kOperatorControllerPort);

    private final Shooter shooter = new Shooter();
    private final Feeder feeder = new Feeder();
    private final Conveyor conveyor = new Conveyor();
    private final IntakeCover intakeCover = new IntakeCover();
    private final Intake intake = new Intake();
    private final ClimberBrake climberBrake = new ClimberBrake();
    private final Climber climber = new Climber(operatorController);

    // Drive train and driver controller
    private final Joystick leftJoystick = new Joystick(0);
    private final Joystick rightJoystick = new Joystick(1);
    // private final GTADrive drive = new GTADrive(driverController, climber);
    private final TankDrive m_drive = new TankDrive();
    private final Limelight limelight = new Limelight();

    public RobotContainer() {
        configureButtonBindings();

        // Make sure the limelight's LED is off when we turn on
        turnOffLimelightLED();

        m_drive.setDefaultCommand(
                new RunCommand(() -> m_drive.drive(
                        leftJoystick.getY(),
                        rightJoystick.getY()), m_drive));

        climber.setDefaultCommand(new Climb(climber, intakeCover, climberBrake));
        // Add commands to the autonomous command chooser

        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto Routine", autoSelection);
    }

    private void configureButtonBindings() {
        /** AUTOMATIC OPERATION (using limelight) */
        // Driver can auto aim to target

        /** MANUAL OPERATION */
        final Trigger shooterOffButton = operatorController.x();
        final Trigger setTargetFar = operatorController.y();
        final Trigger setTargetTBD = operatorController.b();
        final Trigger setTargetNear = operatorController.a();
        final Trigger bumpTargetSpeeds = operatorController.rightBumper();

        // Create fake button to correspond to right trigger pressed
        final Trigger shootButton = operatorController.rightTrigger(0.5);

        // Create fake buttons to correspond to right joystick up / down
        final Trigger cargoLoadControl = operatorController.axisGreaterThan(1, .25);
        final Trigger cargoEjectControl = operatorController.axisLessThan(1, -0.25);

        final Trigger unlockClimberButton = operatorController.back();
        final Trigger lockClimberButton = operatorController.start();

        // Create fake buttons from POV Dpad Values
        final Trigger closeIntakeCoverButton = operatorController.povUp();
        final Trigger openIntakeCoverButton = operatorController.povDown();

        /**
         * SET SHOOTING TARGET
         * Setting the shooting target will update the shooter motor setpoint
         */
        bumpTargetSpeeds.whileTrue(new InstantCommand(shooter::setBumped, shooter))
                .onFalse(new InstantCommand(shooter::setNormal, shooter));

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
                        // If the climber is UNLOCKED, do not close intake
                        new InstantCommand(),
                        // Check if the climber locked
                        climber::isLocked));

        /** LOCK AND UNLOCK CLIMBER AND BRAKE */
        unlockClimberButton.onTrue(new SequentialCommandGroup(
                new InstantCommand(climber::unlockClimber, climber),
                new InstantCommand(climberBrake::unlockClimber, climberBrake),
                // Turn off LL when climbing
                new InstantCommand(() -> limelight.setLEDMode(LedMode.kforceOff))));

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
        return new InstantCommand();
    }
}
