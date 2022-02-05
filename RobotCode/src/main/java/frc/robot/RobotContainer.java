// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ArcadeDriver;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(Constants.OIConstants.kOperatorControllerPort);
  private final DriveTrain m_robotDrive = new DriveTrain();
  private final Shooter m_shooter = new Shooter();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // m_robotDrive.setDefaultCommand(
    // // A split-stick arcade command, with forward/backward controlled by the left
    // // hand, and turning controlled by the right.
    // new ArcadeDriver(
    // m_robotDrive, m_driverController::getRightX, m_driverController::getLeftY));

  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // Spin up the shooter when the 'A' button is pressed
    new JoystickButton(m_operatorController, Button.kA.value)
        .whenPressed(new InstantCommand(m_shooter::enable, m_shooter));

    // Turn off the shooter when the 'B' button is pressed
    new JoystickButton(m_operatorController, Button.kB.value)
        .whenPressed(new InstantCommand(m_shooter::disable, m_shooter));

    // Run the feeder when the 'X' button is held, but only if the shooter is at
    // speed
    new JoystickButton(m_operatorController, Button.kX.value)
        .whenPressed(
            new ConditionalCommand(
                // Run the feeder
                new InstantCommand(m_shooter::runFeeder, m_shooter),
                // Do nothing
                new InstantCommand(),
                // Determine which of the above to do based on whether the shooter has
                // reached the desired speed
                // m_shooter::atSetpoint
                () -> true))
        .whenReleased(new InstantCommand(m_shooter::stopFeeder, m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
