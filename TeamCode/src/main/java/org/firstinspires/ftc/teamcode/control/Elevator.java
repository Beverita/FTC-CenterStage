package org.firstinspires.ftc.teamcode.control;

import lombok.Getter;

/**
 * Interface for communicating with the elevator.
 */
public interface Elevator {

    enum ElevatorLevel {
        /**
         * The lowest level of the elevator (at the base of the robot)
         */
        BASE(0),
        /**
         * The low level of the elevator (slightly above the shortest bar)
         */
        LOW(37.2),
        /**
         * The middle level of the elevator (slightly above the medium bar)
         */
        MIDDLE(60.8),
        /**
         * The highest level of the elevator (slightly above the tallest bar)
         */
        HIGH(88),

        /**
         * Maximum allowed level the elevator might go to
         */
        MAX(90);

        @Getter
        private double height = 0;


        ElevatorLevel(double height) {
            this.height = height;
        }
    }

    /**
     * Updates the motor power to match the target. Must be called in a loop.
     */
    void update();

    /**
     * Sets the target position in encoder ticks
     * @param target the desired target the elevator should maintain
     */
    void setTarget(int target);

    /**
     * Sets the target position based on the levels of the junctions
     * @param level the desired level
     */
    void setTarget(ElevatorLevel level);

    /**
     * Returns the target position the elevator was requested to maintain
     * @return the target position in encoder ticks
     */
    int getTarget();

    /**
     * Returns the current position of the elevator
     * @return the current position in encoder ticks
     */
    int getCurrentPosition();

    /**
     * Checks if bypass limitations are active or not
     * @return true if the bypass is enabled, false otherwise
     */
    boolean isBypass();

    /**
     * Enables or disabled elevator bypass limitations
     * @param bypass true if the bypass should be enabled, false otherwise
     */
    void setBypass(boolean bypass);
}
