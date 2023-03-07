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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class TankDrive extends SubsystemBase {
    private Joystick leftJoystick = new Joystick(0);
    private Joystick rightJoystick = new Joystick(1);

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

    // Construct TankDrive
    public TankDrive() {

        // We need to restore the motors to defaults every time
        // they're initialized, otherwise the encoders go bezerk
        left1.restoreFactoryDefaults();
        left2.restoreFactoryDefaults();
        right1.restoreFactoryDefaults();
        right2.restoreFactoryDefaults();

        // Reverse the left motor
        m_leftMotors.setInverted(true);

        // Reset sensor values when starting
        reset();

        // Let's name the sensors on the LiveWindow
        addChild("Drive", m_drive);
        addChild("Gyro", m_gyro);

    }

    /** The log method puts interesting information to the SmartDashboard. */
    public void log() {
        SmartDashboard.putNumber("Raw encoder read", m_left1Encoder.getPosition());
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

        m_drive.tankDrive(left, right);
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