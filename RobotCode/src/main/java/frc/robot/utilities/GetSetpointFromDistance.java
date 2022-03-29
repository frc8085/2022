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
    private double setpointRPM;
    private double distanceFeet;

    public double setpointFromDistance(double distanceInches) {
        distanceFeet = distanceInches / 12;
        setpointRPM = 53.255 * Math.pow(distanceFeet, 2) - 336.68 * distanceFeet + 4133.5;
        return setpointRPM;
    }
}