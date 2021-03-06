// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Conveyor extends SubsystemBase {
    /** Creates a new Conveyor. */

    // Conveyor motors
    private final CANSparkMax conveyor1 = new CANSparkMax(kConveyorMotorPort1, MotorType.kBrushless);
    private final CANSparkMax conveyor2 = new CANSparkMax(kConveyorMotorPort2, MotorType.kBrushless);

    // Run the conveyor
    public void runConveyor() {
        conveyor1.set(kConveyorSpeed);
        conveyor2.set(-kConveyorSpeed);
    }

    // Reverse the conveyor
    public void reverseConveyor() {
        conveyor1.set(-kConveyorSpeed);
        conveyor2.set(kConveyorSpeed);
    }

    // Stop the conveyor
    public void stopConveyor() {
        conveyor1.set(0);
        conveyor2.set(0);
    }
}
