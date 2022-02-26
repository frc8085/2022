package frc.robot.subsystems;

import com.fasterxml.jackson.databind.jsontype.impl.SubTypeValidator;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  // Define a variable for our color sensor
  ColorSensorV3 colorSensor;

  public ColorSensor() {

  }

  public String getColor() {
    Color color = colorSensor.getColor();
    return color.toString();
  }
}
