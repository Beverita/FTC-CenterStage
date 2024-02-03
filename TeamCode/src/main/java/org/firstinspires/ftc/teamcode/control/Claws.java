package org.firstinspires.ftc.teamcode.control;

/**
 * Every claw's implementor is provided with this method.
 */
public interface Claws {
    /**
     * Invokes the claws to close or open
     *
     * @return true if the claws opened or false if the claws closed
     */
    boolean useClaws();

    /**
     * Closes or opens the claws at the user's discretion
     * @param state true if the claws should open, false if the claws should close
     */
    void useClaws(boolean state);

    /**
     * Returns the claws state
     * @return true if the claws are open, false if the claws are close
     */

    boolean useArm();

    void useArm(boolean state);

    void usePlaneLauncher();

    boolean isClaws();
}
