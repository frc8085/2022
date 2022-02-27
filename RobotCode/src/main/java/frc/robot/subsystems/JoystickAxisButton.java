package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Turn a Joystick Axis into a button
 */
public class JoystickAxisButton extends Button {

    private double m_joystick_value;
    private double THRESHOLD = 0.5;

    /**
     * Create a button for triggering commands off a joystick's analog axis
     * 
     * @param joystick_value The value read from the joystick axis
     */
    public JoystickAxisButton(double joystick_value) {
        m_joystick_value = joystick_value;
    }

    /**
     * Create a button for triggering commands off a joystick's analog axis
     * 
     * @param joystick_value The value read from the joystick axis
     * @param threshold      The threshold to trigger above (positive) or below
     *                       (negative)
     */
    public JoystickAxisButton(double joystick_value, double threshold) {
        m_joystick_value = joystick_value;
        THRESHOLD = threshold;
    }

    /**
     * Set the value above which triggers should occur (for positive thresholds)
     * or below which triggers should occur (for negative thresholds)
     * The default threshold value is 0.5
     * 
     * @param threshold the threshold value (1 to -1)
     */
    public void setThreshold(double threshold) {
        THRESHOLD = threshold;
    }

    /**
     * Get the defined threshold value.
     * 
     * @return the threshold value
     */
    public double getThreshold() {
        return THRESHOLD;
    }

    public boolean get() {
        if (THRESHOLD < 0) {
            return m_joystick_value < THRESHOLD; // Return true if axis value is less than negative
                                                 // threshold
        } else {
            return m_joystick_value > THRESHOLD; // Return true if axis value is greater than
                                                 // positive threshold
        }
    }

}