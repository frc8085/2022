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
import frc.robot.commands.ArcadeDriver;
import frc.robot.commands.ExampleCommand;
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
import edu.wpi.first.networktables.NetworkTableEntry;

public class RobotContainer {
  // The robot's subsystems and commands
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // Drive train and driver controller
  private final XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
  private final DriveTrain m_robotDrive = new DriveTrain();

  // OPerator controller and subsystems
  private final XboxController m_operatorController = new XboxController(Constants.OIConstants.kOperatorControllerPort);
  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder();
  private final Intake m_intake = new Intake();

  // By default set us to shoot at the High Near target
  private int shootMode = Constants.OIConstants.kTargetHighNear;
  private NetworkTableEntry shootingModeDisplay;
  // Translate the shooting modes to descriptions
  private static final Map<Integer, String> shootingMode = new HashMap<Integer, String>() {
    {
      put(Constants.OIConstants.kTargetHighNear, "High->Near");
      put(Constants.OIConstants.kTargetHighFar, "High---------------->Far");
      put(Constants.OIConstants.kTargetLow, "Low");
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
        .withPosition(1, 1)
        .withSize(2, 1)
        .getEntry();

  }

  private void configureButtonBindings() {
    // Create some buttons
    final JoystickButton shootButton = new JoystickButton(m_operatorController, Axis.kRightTrigger.value);
    final JoystickButton setTargetHighNearButton = new JoystickButton(m_operatorController, Button.kY.value);
    final JoystickButton setTargetHighFarFButton = new JoystickButton(m_operatorController, Button.kB.value);
    final JoystickButton setTargetLowButton = new JoystickButton(m_operatorController, Button.kA.value);
    final JoystickButton cargoTakeInButton = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
    final JoystickButton cargoEjectButton = new JoystickButton(m_operatorController, Axis.kLeftTrigger.value);

    // TODO. How do we map DPAD buttons??
    // final JoystickButton dpadUp = new JoystickButton(m_operatorController, 5);
    // final JoystickButton dpadRight = new JoystickButton(m_operatorController, 6);
    // final JoystickButton dpadDown = new JoystickButton(m_operatorController, 7);
    // final JoystickButton dpadLeft = new JoystickButton(m_operatorController, 8);

    // SET SHOOTING TARGET
    setTargetHighNearButton.whenPressed(
        new InstantCommand(() -> shootMode = Constants.OIConstants.kTargetHighNear)
            .andThen(
                () -> shootingModeDisplay.setString(shootingMode.get(shootMode))));

    setTargetHighFarFButton.whenPressed(
        new InstantCommand(() -> shootMode = Constants.OIConstants.kTargetHighFar)
            .andThen(
                () -> shootingModeDisplay.setString(shootingMode.get(shootMode))));

    setTargetLowButton.whenPressed(
        new InstantCommand(() -> shootMode = Constants.OIConstants.kTargetLow)
            .andThen(
                () -> shootingModeDisplay.setString(shootingMode.get(shootMode))));

    // SHOOT ACTION
    // Shoot High Near
    shootButton.whenPressed(
        new ConditionalCommand(
            new ShootHighNear(m_intake, m_feeder, m_shooter)
                .andThen(() -> System.out.println("SHOOTING HIGH NEAR")),
            new InstantCommand(),
            () -> shootMode == Constants.OIConstants.kTargetHighNear));

    // Shoot High Far
    shootButton.whenPressed(
        new ConditionalCommand(
            new ShootHighFar(m_intake, m_feeder, m_shooter)
                .andThen(() -> System.out.println("SHOOTING HIGH FAR")),

            new InstantCommand(),
            () -> shootMode == Constants.OIConstants.kTargetHighFar));

    // Shoot Low
    shootButton.whenPressed(
        new ConditionalCommand(
            new ShootLow(m_intake, m_feeder, m_shooter)
                .andThen(() -> System.out.println("SHOOTING LOW")),

            new InstantCommand(),
            () -> shootMode == Constants.OIConstants.kTargetLow));

    // // Spin up the shooter when the 'A' button is pressed
    // new JoystickButton(m_operatorController, Button.kA.value)
    // .whenPressed(new InstantCommand(m_shooter::setSetpoint, m_shooter));

    // // Turn off the shooter when the 'B' button is pressed
    // new JoystickButton(m_operatorController, Button.kB.value)
    // .whenPressed(new InstantCommand(m_shooter::stopShooter, m_shooter));

    // // Run intake when the 'Left bumper' held down
    // new JoystickButton(m_operatorController, Button.kLeftBumper.value)
    // .whenPressed(new InstantCommand(m_intake::runIntake, m_intake))
    // .whenReleased(new InstantCommand(m_intake::stopIntake, m_intake));

    // // Run the feeder when the 'X' button is held, but only if the shooter is at
    // // speed
    // new JoystickButton(m_operatorController, Button.kX.value)
    // .whenPressed(
    // new ConditionalCommand(
    // // Run the feeder
    // new InstantCommand(m_feeder::runFeeder, m_feeder),
    // // Do nothing
    // new InstantCommand(),
    // // Determine which of the above to do based on whether the shooter has
    // // reached the desired speed
    // m_shooter::atSetpoint))
    // .whenReleased(new InstantCommand(m_feeder::stopFeeder, m_feeder));
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
