package org.firstinspires.ftc.teamcode.control;

/**
 * Every implementor has to be able to operate the drive train of the robot with this method.
 */
public interface Drivetrain {

    /**
     * Moves the robot in the specified direction
     *
     * @param axial   the axial direction of the robot
     * @param lateral the lateral direction of the robot
     * @param yaw     the yaw direction of the robot
     */
    void drive(double axial, double lateral, double yaw);

    /**
     * Moves the robot by specifying the power of each motor
     *
     * @param leftFront  the power of the left front motor
     * @param rightFront the power of the right front motor
     * @param leftBack   the power of the left back motor
     * @param rightBack  the power of the right back motor
     */
    void drive(double leftFront, double rightFront, double leftBack, double rightBack);
}
