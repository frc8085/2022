// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.html.HTMLEditorKit.LinkController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 */
public class Robot extends TimedRobot {
  private static final int motorPort1 = 0;
  private static final int motorPort3 = 1;
  private static final int motorPort2 = 2;
  private static final int motorPort4 = 3;
  private static final int motorIntake = 4;
  private static final int DJoystickPort = 0;
  private static final int OJoystickPort = 1;

  private MotorController m_motor1;
  private MotorController m_motor2;
  private MotorController m_motor3;
  private MotorController m_motor4;
  private MotorController m_intake;
  private Joystick D_joystick;
  private Joystick O_joystick;

  @Override
  public void robotInit() {
    m_motor1 = new PWMSparkMax(motorPort1);
    m_motor3 = new PWMSparkMax(motorPort3);
    m_motor2 = new PWMSparkMax(motorPort2);
    m_motor4 = new PWMSparkMax(motorPort4);
    m_intake = new PWMSparkMax(motorIntake);
    D_joystick = new Joystick(DJoystickPort);
    O_joystick = new Joystick(OJoystickPort);
  }

  @Override
  public void teleopPeriodic() {
    //Drive train motors, defined by D_joystick (AKA Driver Controller)
    //Moving backwards (right trigger) not working
    /*m_motor1.set(D_joystick.getRawAxis(3));
    m_motor3 = m_motor1;
    m_motor2.set(D_joystick.getRawAxis(3)*-1);
    m_motor4 = m_motor2;*/
    //Moving fowards (left trigger) working
    /*m_motor1.set(D_joystick.getRawAxis(2)*-1);
    m_motor3=m_motor1;
    m_motor2.set(D_joystick.getRawAxis(2));
    m_motor4 = m_motor2; */

    m_motor1.set(D_joystick.getY());
    m_motor3 = m_motor1;
    m_motor2.set(D_joystick.getY()*-1);
    m_motor4 = m_motor2;
    //m_motor2.set(D_joystick.getX()*-1);

    //Turning Left
    //m_motor1.set(D_joystick.getX());
    //m_motor2.set(D_joystick.getX());

    //Intake motor(s), defined by O_joystick (AKA Operator Controller)
    m_intake.set(O_joystick.getY());



  }
}
