package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;

public class GTADrive extends SubsystemBase {
    private XboxController m_driverController;
    private double speed;
    private double turnRotation;

    // Left motors
    private final CANSparkMax left1 = new CANSparkMax(kLeftMotor1Port, MotorType.kBrushless);
    private final CANSparkMax left2 = new CANSparkMax(kLeftMotor2Port, MotorType.kBrushless);

    // Right motors
    private final CANSparkMax right1 = new CANSparkMax(kRightMotor1Port, MotorType.kBrushless);
    private final CANSparkMax right2 = new CANSparkMax(kRightMotor2Port, MotorType.kBrushless);

    // Create motor groups
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(left1, left2);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(right1, right2);

    // Encoders
    private final RelativeEncoder m_left1Encoder = left2.getEncoder();
    private final RelativeEncoder m_right1Encoder = right2.getEncoder();

    // Gyro
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

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
        left1.restoreFactoryDefaults();
        left2.restoreFactoryDefaults();
        right1.restoreFactoryDefaults();
        right2.restoreFactoryDefaults();

        // Reverse the lefts motor and left motor encoder
        m_leftMotors.setInverted(true);

        // Reset sensor values when starting
        reset();

        // Let's name the sensors on the LiveWindow
        addChild("Drive", m_drive);
        addChild("Gyro", m_gyro);
    }

    /** The log method puts interesting information to the SmartDashboard. */
    public void log() {
        SmartDashboard.putNumber("Left Motor Revs", m_left1Encoder.getPosition());
        SmartDashboard.putNumber("Right Motor Revs", m_right1Encoder.getPosition());

        SmartDashboard.putNumber("Left Inches", getLeftEncoderDistanceInches());
        SmartDashboard.putNumber("Right Inches", getRightEncoderDistanceInches());

        SmartDashboard.putNumber("Distance traveled", getDistance());

        SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    }

    /** Call log method every loop. */
    @Override
    public void periodic() {
        log();
    }

    // Tank drive for autonomous
    public void drive(double left, double right) {

        // TODO: Fix this hack using a better util
        int directionL = left >= 0 ? 1 : -1;
        int directionR = right >= 0 ? 1 : -1;

        // Clamps the output to between 0.4 and 0.5
        double speedL = MathUtil.clamp(left, 0.4, 0.6);
        double speedR = MathUtil.clamp(right, 0.4, 0.6);

        m_drive.tankDrive(speedL * directionL, speedR * directionR);
    }

    // Turn
    public void turn(double distance) {
        m_drive.tankDrive(-distance, distance);
    }

    public void driveRobot() {
        double leftTrigger = m_driverController.getLeftTriggerAxis();
        double rightTrigger = m_driverController.getRightTriggerAxis();
        boolean leftBumper = m_driverController.getLeftBumper();

        // Transform the turn rotation based on Right Joystick X
        turnRotation = Math.pow(m_driverController.getRightX(), 3) * -0.5;

        if (isStopped(leftTrigger, rightTrigger)) {
            // Even if it's stopped, let it turn
            m_drive.curvatureDrive(0, turnRotation, true);
        } else {
            if (leftBumper) {
                speed = .1;

                speed = applyDirection(Math.abs(speed), leftTrigger, rightTrigger);
                m_drive.curvatureDrive(speed, turnRotation, true);

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
        m_left1Encoder.setPosition(0);
        m_right1Encoder.setPosition(0);
    }

    /**
     * Get the average distance (inches) of the encoders since the last reset.
     *
     * @return The distance driven (average of left and right encoders).
     */
    public double getDistance() {
        return (getLeftEncoderDistanceInches() + getRightEncoderDistanceInches()) / 2;
    }

    public double getTurnedInches() {
        return (getLeftEncoderDistanceInches() + -1 * getRightEncoderDistanceInches()) / 2;
    }

    public double getLeftEncoderDistanceInches() {
        double leftMotorRevolutions = m_left1Encoder.getPosition() * kReverse;
        double leftDistanceInches = leftMotorRevolutions * kInchesPerMotorRevolution;
        return leftDistanceInches;
    }

    public double getRightEncoderDistanceInches() {
        double rightMotorRevolutions = m_right1Encoder.getPosition();
        double rightDistanceInches = rightMotorRevolutions * kInchesPerMotorRevolution;
        return rightDistanceInches;
    }

}