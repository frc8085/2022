package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;

public class GTADrive extends SubsystemBase {
  private XboxController m_driverController;
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

  // Encoders
  private final Encoder m_leftEncoder = new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1]);
  private final Encoder m_rightEncoder = new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1]);

  private final RelativeEncoder left1Encoder = left1.getEncoder();

  // Gyro
  private final AnalogGyro m_gyro = new AnalogGyro(DriveConstants.kGyroChannel);

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

  public GTADrive(XboxController driverController) {
    m_driverController = driverController;

    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    m_leftMotors.setInverted(true);

    // Let's name the sensors on the LiveWindow
    addChild("Drive", m_drive);
    addChild("Left Encoder", m_leftEncoder);
    addChild("Right Encoder", m_rightEncoder);
    addChild("Gyro", m_gyro);
  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {

    // Unit = Revolutions (of the Motor)
    // * 10.75
    SmartDashboard.putNumber("Left1 Position", left1Encoder.getPosition());

    // SmartDashboard.putNumber("Left Distance", m_leftEncoder.getDistance());
    // SmartDashboard.putNumber("Right Distance", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Speed", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Speed", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
  }

  // Tank drive for autonomous
  public void drive(double left, double right) {
    SmartDashboard.putNumber("left wanted", left);
    SmartDashboard.putNumber("right wanted", right);
    SmartDashboard.putNumber("left gotten", left1.get());
    SmartDashboard.putNumber("right gotten", right1.get());
    m_drive.tankDrive(left, right);
  }

  public void driveRobot() {
    double leftTrigger = m_driverController.getLeftTriggerAxis();
    double rightTrigger = m_driverController.getRightTriggerAxis();

    // Transform the turn rotation based on Right Joystick X
    turnRotation = Math.pow(m_driverController.getRightX(), 3) * -0.5;

    if (isStopped(leftTrigger, rightTrigger)) {
      // Even if it's stopped, let it turn
      m_drive.curvatureDrive(0, turnRotation, true);
    } else {
      speed = m_driverController.getLeftY();
      // Up is fast. Down is slow. Multiply value by .5 so that the max range is 0.5
      // to -0.5
      speed *= -0.5;

      // rescale joystick so zero is half speed
      speed += 0.5;

      // change factor so that we can readjust scale such that slowest is .2, neutral
      // is .6, and fastest is 1
      speed *= .8;
      speed += .2;

      speed = applyDirection(Math.abs(speed), leftTrigger, rightTrigger);
      m_drive.curvatureDrive(speed, turnRotation, true);
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
    return (isForward(leftTrigger, rightTrigger)) ? speed * -1 : speed * 1;
  }

  /**
   * Get the robot's heading.
   *
   * @return The robots heading in degrees.
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /** Reset the robots sensors to the zero states. */
  public void reset() {
    m_gyro.reset();
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Get the average distance of the encoders since the last reset.
   *
   * @return The distance driven (average of left and right encoders).
   */
  public double getDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
  }

  public double getLeftEncoderDistance() {
    return m_leftEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return m_rightEncoder.getDistance();
  }
}