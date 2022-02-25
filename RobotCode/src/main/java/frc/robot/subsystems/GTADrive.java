package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;

public class GTADrive extends SubsystemBase {
  private XboxController stick;
  private int leftTriggerPort;
  private int rightTriggerPort;
  private int leftYAxis;
  private int rightXAxis;
  private double speed;
  private double turnRotation;

  // Left motors
  private final CANSparkMax left1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax left2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

  // Right motors
  private final CANSparkMax right1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax right2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  // Create motor groups
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(left1, left2);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(right1, right2);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(
      m_leftMotors, m_rightMotors);

  /**
   * Construct a GTADrive.
   * Right trigger ---------- Forward
   * Left trigger ----------- Reverse
   * Left Joystick Up/Down -- Faster
   * Left Joystick Down ----- Slower
   * Right Joystick Left ---- Move left
   * Right Josytick Right --- Move right
   */

  public GTADrive(XboxController m_driverController) {
    stick = m_driverController;

    leftTriggerPort = Axis.kLeftTrigger.value;
    rightTriggerPort = Axis.kRightTrigger.value;

    leftYAxis = Axis.kLeftY.value;
    rightXAxis = Axis.kRightX.value;
  }

  public void driveRobot() {
    double leftTrigger = stick.getRawAxis(leftTriggerPort);
    double rightTrigger = stick.getRawAxis(rightTriggerPort);
    if (isStopped(leftTrigger, rightTrigger)) {
      m_drive.tankDrive(0, 0);
    } else {

      speed = stick.getRawAxis(leftYAxis);
      speed *= -.5;
      speed += .5;
      speed = applyDirection(Math.abs(speed), leftTrigger, rightTrigger);

      turnRotation = stick.getRawAxis(rightXAxis) * -1;
      m_drive.arcadeDrive(speed, turnRotation);

      SmartDashboard.putNumber("Speed", speed);
      SmartDashboard.putNumber("Rotation", turnRotation);
    }
  }

  /**
   * Check if the joystick is stopped based on trigger values (both or zero)
   * 
   * @return Whether the trigger values mean it is stopped
   */
  private boolean isStopped(double leftTrigger, double rightTrigger) {
    boolean isReverse = leftTrigger <= 0;
    boolean isForward = rightTrigger <= 0;
    return isReverse && isForward || (!isReverse && !isForward);
  }

  /**
   * Check if the trigger is going forward (not stopped & right trigger depressed)
   * 
   * @return Whether the trigger values mean it is going forward
   */
  private boolean isForward(double leftTrigger, double rightTrigger) {
    return !isStopped(leftTrigger, rightTrigger) && rightTrigger <= 0;
  }

  /**
   * Apply the direction to the the speed to make it either forward or reverse.
   * 
   * @param speed The speed given by the joystick
   * @return The value of the speed with the direction applied
   */
  private double applyDirection(double speed, double leftTrigger, double rightTrigger) {
    return (isForward(leftTrigger, rightTrigger)) ? speed * 1 : speed * -1;
  }

}