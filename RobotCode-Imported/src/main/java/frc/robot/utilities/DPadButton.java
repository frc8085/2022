
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Shoot;

/**
 * Turns DPAD POV into buttons
 */

public class DPadButton extends Button {
    private boolean DEBUG_MODE = false;
    private int dPadDegree;
    private XboxController controller;
    private NetworkTableEntry controlsDisplay;
    private String label = "";

    public enum Value {
        kDPadRight, kDPadUpRight, kDPadUp, kDPadUpLeft,
        kDPadLeft, kDPadDownLeft, kDPadDown, kDPadDownRight,
    }

    /**
     * Creates a dpad button
     * 
     * @param controller the controller to attach the button to
     * @param value      the dpad value
     */

    public DPadButton(String label, XboxController controller, Value value) {
        this.controller = controller;
        this.label = label;
        switch (value) {
            case kDPadRight:
                this.dPadDegree = 90;
                break;
            case kDPadUpRight:
                this.dPadDegree = 45;
                break;
            case kDPadUp:
                this.dPadDegree = 0;
                break;
            case kDPadUpLeft:
                this.dPadDegree = 315;
                break;
            case kDPadLeft:
                this.dPadDegree = 270;
                break;
            case kDPadDownLeft:
                this.dPadDegree = 225;
                break;
            case kDPadDown:
                this.dPadDegree = 180;
                break;
            case kDPadDownRight:
                this.dPadDegree = 135;
                break;
            default:
                throw new AssertionError("Illegal value" + value);
        }

        if (DEBUG_MODE) {
            SmartDashboard.putBoolean(label, false);
        }
    }

    @Override
    public boolean get() {
        boolean pressed = controller.getPOV() == dPadDegree;
        if (DEBUG_MODE) {
            SmartDashboard.putBoolean(label, pressed);
        }
        return pressed;
    }
}