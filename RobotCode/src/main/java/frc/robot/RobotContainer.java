// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ExampleSubsystem;

import java.util.HashMap;
import java.util.Map;

// Inputs
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArcadeDriver;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootHighFar;
import frc.robot.commands.ShootHighNear;
import frc.robot.commands.ShootLow;

// Subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// Displays
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

public class RobotContainer {
    // The robot's subsystems and commands
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    // Drive train and driver controller
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final DriveTrain m_robotDrive = new DriveTrain();

    // OPerator controller and subsystems
    private final XboxController m_operatorController = new XboxController(
            OIConstants.kOperatorControllerPort);
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final Intake m_intake = new Intake();

    /**
     * Shooting Mode definition
     * shootMode is an integer corresponding to the three types of tragets
     * (High Near, High Far, Low)
     * shootingMode translates the integer into a string so we can display it
     * in the Shuffleboard entry shottingModeDisplay
     */
    private NetworkTableEntry shootingModeDisplay;
    private int shootMode = OIConstants.kShooterOff;
    private static final Map<Integer, String> shootingMode = new HashMap<Integer, String>() {
        {
            put(OIConstants.kShooterOff, "Shooting mode not selected");
            put(OIConstants.kTargetHighNear, "High->Near");
            put(OIConstants.kTargetHighFar, "High----------------------------->Far");
            put(OIConstants.kTargetLow, "Low");
        }
    };

    public RobotContainer() {
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
                // Forward/backward controlled by the left joystick
                // Turning controlled by the right joystick
                new ArcadeDriver(
                        m_robotDrive, m_driverController::getRightX, m_driverController::getLeftY));

        shootingModeDisplay = Shuffleboard.getTab("Shooter")
                .add("Shooting Mode", shootingMode.get(shootMode))
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();

    }

    private void configureButtonBindings() {
        // Create some buttons
        final JoystickButton shootButton = new JoystickButton(m_operatorController, Axis.kRightTrigger.value);
        final JoystickButton shooterOffButton = new JoystickButton(m_operatorController, Button.kX.value);
        final JoystickButton setTargetHighNearButton = new JoystickButton(m_operatorController, Button.kY.value);
        final JoystickButton setTargetHighFarButton = new JoystickButton(m_operatorController, Button.kB.value);
        final JoystickButton setTargetLowButton = new JoystickButton(m_operatorController, Button.kA.value);
        final JoystickButton cargoTakeInButton = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
        final JoystickButton cargoEjectButton = new JoystickButton(m_operatorController, Axis.kLeftTrigger.value);

        // TODO. How do we map DPAD buttons??
        // final JoystickButton dpadUp = new JoystickButton(m_operatorController, 5);
        // final JoystickButton dpadRight = new JoystickButton(m_operatorController, 6);
        // final JoystickButton dpadDown = new JoystickButton(m_operatorController, 7);
        // final JoystickButton dpadLeft = new JoystickButton(m_operatorController, 8);

        /**
         * SET SHOOTING TARGET
         * Setting the shooting target will update the shooter motor setpoint
         */

        setTargetHighNearButton.whenPressed(
                new InstantCommand(() -> setShootingMode(OIConstants.kTargetHighNear)));
        setTargetHighFarButton.whenPressed(
                new InstantCommand(() -> setShootingMode(OIConstants.kTargetHighFar)));
        setTargetLowButton.whenPressed(
                new InstantCommand(() -> setShootingMode(OIConstants.kTargetLow)));

        /**
         * SHOOT ACTIOIN
         * Shooting will run the feeder, but only if the shooter is up to speed.
         * The speed depends on which shooting target mode we are in.
         * Releasing the shoot trigger will stop the feeder.
         */

        shootButton.whenPressed(new Shoot(m_intake, m_feeder, m_shooter))
                .whenReleased(new InstantCommand(m_feeder::stopFeeder, m_feeder));

        shooterOffButton.whenPressed(new InstantCommand(m_shooter::stopShooter, m_shooter)
                .andThen(() -> setShootingMode(OIConstants.kShooterOff)));

        // // Run intake when the 'Left bumper' held down
        // new JoystickButton(m_operatorController, Button.kLeftBumper.value)
        // .whenPressed(new InstantCommand(m_intake::runIntake, m_intake))
        // .whenReleased(new InstantCommand(m_intake::stopIntake, m_intake));

    }

    private void setShootingMode(int mode) {
        shootMode = mode;
        shootingModeDisplay.setString(shootingMode.get(shootMode));
        m_shooter.setSetpoint(ShooterConstants.kShooterTargetRPM);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot}
     * class. @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoCommand;
    }
}
