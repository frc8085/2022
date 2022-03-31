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
        double d = dInches;
        double setpointRPM = dInches < 100 ? 3550 : 0.1607 * Math.pow(d, 2) - 28.274 * d + 4991.1;
        return setpointRPM;
    }
}