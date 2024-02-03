package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import lombok.Getter;
import lombok.Setter;

/**
 * This class provides enhanced functionality for TeleOp.
 * All the important parts are already initialized (gamepads and hardware).
 * You can skip the start and init phases and jump straight into the running phase. 2 debug buttons are provided for the
 * 2 controllers, and you can assign commands to them. They act as toggle buttons. Everything that happens during the OpMode
 * is in the {@link TeleOpControl#run()} method and this is the only method you have to implement.
 * Telemetry, gamepads and debug keys are automatically updated unless stated otherwise.
 * {@link Telemetry#update()} is called in the main loop after all method calls. That includes {@link TeleOpControl#run()},
 * {@link TeleOpControl#debugMainGamepad()} and {@link TeleOpControl#debugMainGamepad()} and {@link  TeleOpControl#debugSecondaryGamepad()}. Only after the execution of
 * these 3 methods is the telemetry updated.
 */

public abstract class

TeleOpControl extends LinearOpMode implements Drivetrain {
    /**
     * Main gamepad or the "driver gamepad". Assigned by default to gamepad1 of the LinearOpMode.
     *
     * @see TeleOpControl#invertedGamepads
     */
    protected GamepadEx mainGamepad;
    /**
     * Secondary gamepad usually for things that are not driving related.
     * Assigned by default to gamepad2 of the LinearOpMode
     *
     * @see TeleOpControl#invertedGamepads
     */
    protected GamepadEx secondaryGamepad;

    /**
     * Should gamepad1 and gamepad2 be reverted? Set to true to invert the roles.
     *
     * @see TeleOpControl#mainGamepad
     * @see TeleOpControl#secondaryGamepad
     */
    protected boolean invertedGamepads = false;

    /**
     * Skip initialization and instantly call {@link LinearOpMode#waitForStart()} after {@link TeleOpControl} internal
     * initialization is complete. Defaults to true.
     */
    protected boolean skipInit = true;

    /**
     * Skip the start phase and instantly go to the looping body of {@link LinearOpMode}.
     * {@link TeleOpControl#enhanced_start()} will not be called if set to true. Defaults to true.
     */
    protected boolean skipStart = true;

    /**
     * The robot hardware. This is the only hardware that is initialized by default.
     */
    protected final RobotHardware robotHardware = new RobotHardware(this);

    @Override
    public final void runOpMode() throws InterruptedException {
        overrideSettings();
        robotHardware.initTeleOp();
        mainGamepad = new GamepadEx(gamepad1);
        secondaryGamepad = new GamepadEx(gamepad2);
        robotHardware.getClawsController().useClaws(true);
        robotHardware.getClawsController().useArm(false);
        if (invertedGamepads) {
            GamepadEx aux = secondaryGamepad;
            secondaryGamepad = mainGamepad;
            mainGamepad = aux;
        }


        if (skipInit) waitForStart();
        else {
            enhanced_init();
            telemetry.update();
            while (opModeInInit()) {
                enhanced_init_loop();
                telemetry.update();
            }
        }

        if (!skipStart) {
            if (!isStopRequested()) enhanced_start();
        }

        while (opModeIsActive()) {
            run();

            if (mainGamepad.wasJustReleased(GamepadKeys.Button.Y)) {
                debugMainGamepad();
            }

            if (secondaryGamepad.wasJustReleased(GamepadKeys.Button.Y)) {
                debugSecondaryGamepad();
            }

            //Updates
            telemetry.update();
            mainGamepad.readButtons();
            secondaryGamepad.readButtons();
        }

        if (isStopRequested()) {
            enhanced_stop();
        }
    }

    /**
     * Override this method to change any fields before the INIT phase.
     */
    protected void overrideSettings() {

    }

    /**
     * This method can be overwritten to provide functionality to the debug key on the main gamepad (Y key)
     */
    protected void debugMainGamepad() {

    }

    /**
     * This method can be overwritten to provide functionality to the debug key on the secondary gamepad (Y key)
     */
    protected void debugSecondaryGamepad() {

    }

    /**
     * Override this method to execute initialization code ONCE. Make sure that {@link TeleOpControl#skipInit} is false
     * otherwise this method will never be called.
     * A call to {@link Telemetry#update()} will be made after this method is called.
     */
    protected void enhanced_init() {

    }

    /**
     * Override this method to execute initialization code repeatedly until START is played. Make sure that {@link TeleOpControl#skipInit} is false
     * otherwise this method will never be called.
     * A call to {@link Telemetry#update()} will be made in the while-loop after each call to this method.
     */
    protected void enhanced_init_loop() {

    }

    /**
     * Override this method to execute code ONCE when the opmode is to shut down.
     * Keep in mind this time you will have to call {@link Telemetry#update()} as this is no longer covered by {@link TeleOpControl}.
     */
    protected void enhanced_stop() {

    }

    /**
     * Override this method to execute code ONCE when the OpMode START button is pressed. Keep in mind this time you
     * will have to call {@link Telemetry#update()} as this is no longer covered by {@link TeleOpControl}.
     * For this method to be called make sure that {@link TeleOpControl#skipStart} is set to false.
     */
    protected void enhanced_start() {

    }

    /**
     * Actual body of the OpMode. This will be repeatedly run until STOP is pressed or a crash occurs.
     * In each iteration this method is called before the debug methods are called.
     */
    protected abstract void run();

    @Override
    public void drive(double axial, double lateral, double yaw) {
        double targetLeftFrontPower = axial + lateral + yaw;
        double targetRightFrontPower = axial - lateral - yaw;
        double targetLeftBackPower = axial - lateral + yaw;
        double targetRightBackPower = axial + lateral - yaw;

        //Find max of all powers
        double max = Math.max(Math.abs(targetLeftFrontPower), Math.abs(targetRightFrontPower));
        max = Math.max(max, Math.abs(targetLeftBackPower));
        max = Math.max(max, Math.abs(targetRightBackPower));

        //If max is greater than 1, scale all powers down
        if (max > 1) {
            targetLeftFrontPower /= max;
            targetRightFrontPower /= max;
            targetLeftBackPower /= max;
            targetRightBackPower /= max;
        }

        targetRightBackPower *= powerScale;
        targetRightFrontPower *= powerScale;
        targetLeftFrontPower *= powerScale;
        targetLeftBackPower *= powerScale;

        robotHardware.getLeftFrontMotor().setPower(targetLeftFrontPower);
        robotHardware.getRightFrontMotor().setPower(targetRightFrontPower);
        robotHardware.getLeftBackMotor().setPower(targetLeftBackPower);
        robotHardware.getRightBackMotor().setPower(targetRightBackPower);
    }


    @Override
    public void drive(double leftFront, double rightFront, double leftBack, double rightBack) {
        robotHardware.getLeftFrontMotor().setPower(leftFront);
        robotHardware.getRightFrontMotor().setPower(rightFront);
        robotHardware.getLeftBackMotor().setPower(leftBack);
        robotHardware.getRightBackMotor().setPower(rightBack);
    }

    @Setter
    @Getter
    private double powerScale = 1;
}
