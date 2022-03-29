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
public class GetSetpointFromDistance {
    private double distanceToTarget, kHMaxFeet, kGFeetPerSecondSquared, vX;
    private double setpointRPM;

    public double setpointFromDistance(double distance) {
        this.distanceToTarget = distance;
        kHMaxFeet = 10;
        kGFeetPerSecondSquared = 32.2;
        vX = (distanceToTarget + 1.5) / Math.sqrt(kHMaxFeet / kGFeetPerSecondSquared);
        setpointRPM = vX;

        return setpointRPM;
    }
}