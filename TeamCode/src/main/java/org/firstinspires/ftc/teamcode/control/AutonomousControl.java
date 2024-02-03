package org.firstinspires.ftc.teamcode.control;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Pipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import lombok.Getter;

public abstract class AutonomousControl extends LinearOpMode {

    public OpenCvCamera camera;
    public Pipeline splitAveragePipeline;
    public int camW = 1280;
    public int camH = 720;

    public int zone = 1;

    public int elementZone = 1;

    @Getter
    protected final RobotHardware robotHardware = new RobotHardware(this);


    protected enum ParkingSpot {
        ONE, TWO, THREE
    }

    @Getter
    private ParkingSpot parkingSpot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.initAutonomous();

        robotHardware.initWebcam();

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        splitAveragePipeline = new Pipeline();

        camera.setPipeline(splitAveragePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        while (opModeInInit()){
            elementZone = elementDetection(telemetry);
            telemetry.addData("getMaxDistance", getMaxDistance());
            telemetry.addData("Zone", elementZone);
        }

        initTrajectories();

        if (isStopRequested()) return;

        if(parkingSpot == null) parkingSpot = ParkingSpot.TWO;

        run();
    }

    protected abstract void initTrajectories();

    protected abstract void run();

    private final double TICKS_CENTIMETER = 537.6 / (9.6 * Math.PI);

    protected void driveStraight(double distance) {
        int target = (int) (distance * TICKS_CENTIMETER);

        robotHardware.getLeftFrontMotor().setTargetPosition(target);
        robotHardware.getLeftBackMotor().setTargetPosition(target);
        robotHardware.getRightFrontMotor().setTargetPosition(target);
        robotHardware.getRightBackMotor().setTargetPosition(target);


        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        setAllMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy()) {
            setAllPower(0.5);
        }
        setAllPower(0);

        setAllMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //pos -> right
    //neg -> left
    protected void driveSideways(double distance) {
        int target = (int) (distance * TICKS_CENTIMETER);

        robotHardware.getLeftFrontMotor().setTargetPosition(target);
        robotHardware.getRightFrontMotor().setTargetPosition(-target);
        robotHardware.getLeftBackMotor().setTargetPosition(-target);
        robotHardware.getRightBackMotor().setTargetPosition(target);

        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        setAllMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isBusy()) {
            setAllPower(0.5);
        }
        setAllPower(0);
        setAllMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean isBusy() {
        return (robotHardware.getLeftFrontMotor().isBusy() ||
                robotHardware.getLeftFrontMotor().isBusy() ||
                robotHardware.getRightBackMotor().isBusy() ||
                robotHardware.getRightFrontMotor().isBusy())
                && !isStopRequested();
    }

    private void setAllPower(double power) {
        robotHardware.getLeftFrontMotor().setPower(power);
        robotHardware.getLeftBackMotor().setPower(power);
        robotHardware.getRightFrontMotor().setPower(power);
        robotHardware.getRightBackMotor().setPower(power);
    }

    private void setAllMode(DcMotor.RunMode mode) {
        robotHardware.getLeftFrontMotor().setMode(mode);
        robotHardware.getLeftBackMotor().setMode(mode);
        robotHardware.getRightFrontMotor().setMode(mode);
        robotHardware.getRightBackMotor().setMode(mode);
    }

    public int elementDetection(Telemetry telemetry) {
        zone = splitAveragePipeline.get_element_zone();
        telemetry.addData("Element Zone", zone);
        return zone;
    }

    public double getMaxDistance(){
        return splitAveragePipeline.getMaxDistance();
    }
}
