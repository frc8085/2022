/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

/**
 * A utility class to calculate the shooter motor setpoint given the target's
 * distance
 */
public final class GetSetpointFromDistance {
    public static double setpointFromDistance(double dInches) {
        double setpointRPM = 0.00005 - 0.0058 * Math.pow(dInches, 4) + 1.0687 * Math.pow(dInches, 3)
                - 95.833 * Math.pow(dInches, 2) + 4200.5 * dInches - 68405;
        return setpointRPM;
    }
}