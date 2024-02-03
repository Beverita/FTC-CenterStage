package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 * <p>
 * Button Mappings:
 * <p>
 * Xbox/PS4 Button - Motor
 * X / ▢         - Front Left
 * Y / Δ         - Front Right
 * B / O         - Rear  Right
 * A / X         - Rear  Left
 * The buttons are mapped to match the wheels spatially if you
 * were to rotate the gamepad 45deg°. x/square is the front left
 * ________        and each button corresponds to the wheel as you go clockwise
 * / ______ \
 * ------------.-'   _  '-..+              Front of Bot
 * /   _  ( Y )  _  \                  ^
 * |  ( X )  _  ( B ) |     Front Left   \    Front Right
 * ___  '.      ( A )     /|       Wheel       \      Wheel
 * .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 * |       |                 |                      \
 * '.___.' '.               |          Rear Left    \   Rear Right
 * '.             /             Wheel       \    Wheel
 * \.          .'              (A/X)        \   (B/O)
 * \________/
 * <p>
 * Uncomment the @Disabled tag below to use this opmode.
 */
@Config
@TeleOp(group = "drive")
@Disabled
public class MotorDirectionDebugger extends LinearOpMode {
    public static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "left_front");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "left_back");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "right_front");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
            telemetry.addLine();

            if (gamepad1.x) {
                leftFront.setPower(MOTOR_POWER);
                telemetry.addData("Running Motor", "Front Left pos: %d", leftFront.getCurrentPosition());
            } else if (gamepad1.y) {
                rightFront.setPower(MOTOR_POWER);
                telemetry.addData("Running Motor", "Front Right pos: %d", rightFront.getCurrentPosition());
            } else if (gamepad1.b) {
                rightBack.setPower(MOTOR_POWER);
                telemetry.addData("Running Motor", "Rear Right pos: %d", rightBack.getCurrentPosition());
            } else if (gamepad1.a) {
                leftBack.setPower(MOTOR_POWER);
                telemetry.addData("Running Motor", "Rear Left pos: %d", leftBack.getCurrentPosition());
            } else {
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                telemetry.addData("Running Motor", "None");
            }

            telemetry.update();
        }
    }
}
