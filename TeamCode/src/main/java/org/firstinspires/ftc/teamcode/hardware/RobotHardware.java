package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.Claws;
import org.firstinspires.ftc.teamcode.control.Elevator;
import org.firstinspires.ftc.teamcode.control.MecanumDriveController;
import org.firstinspires.ftc.teamcode.control.limbs.ClawsController;
import org.firstinspires.ftc.teamcode.control.limbs.ElevatorController;
import org.firstinspires.ftc.teamcode.util.constants.DriveConstants;

import java.util.ArrayList;
import java.util.List;

import lombok.Getter;

/**
 * <h1>This class can be used to define all the specific hardware for the Perpetuum Mobile Robot.</h1>
 * This is NOT an OpMode.
 * <br>
 * <p>Used Motors (driving & lift): <a href="https://www.gobilda.com/yellow-jacket-planetary-gear-motors/">5203 Series goBILDA Planetary Gear Motors</a></p>
 * <h2>Motors</h2>
 * <h3>Motors for driving the robot</h3>
 * <pre>Right front motor:                                  <i>"right_front"</i></pre>
 * <pre>Right back motor:                                   <i>"right_back"</i></pre>
 * <pre>Left front motor:                                   <i>"left_front"</i></pre>
 * <pre>Left back motor:                                    <i>"left_back"</i></pre>
 * <h3>Motor for using the elevator</h3>
 * <pre>Left elevator motor:                             <i>"left_elevator"</i></pre>
 * <pre>Right elevator motor:                           <i>"right_elevator"</i></pre>
 * <br>
 * <h2> Servos </h2>
 * <pre> Claw servo:                                        <i> "claw"</i> </pre>
 * <h2>Sensors and misc</h2>
 * <h3>2M Distance Sensors</h3>
 * <pre>Left side sensor                                    <i>"left_2m"</i></pre>
 * <pre>Right side sensor                                   <i>"right_2m</i></pre>
 * <pre>Back sensor                                         <i>"back_2m</i></pre>
 * <h3>Misc</h3>
 * <pre>BNO55IMU Gyroscope                                  <i>"imu"</i></pre>
 */


public class RobotHardware {
    /**
     * The OpMode that requested the hardware map
     */
    private final OpMode opMode;

    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;
    }

    @Getter
    private DcMotorEx leftFrontMotor = null;
    @Getter
    private DcMotorEx leftBackMotor = null;
    @Getter
    private DcMotorEx rightFrontMotor = null;
    @Getter
    private DcMotorEx rightBackMotor = null;
    @Getter
    private DcMotorEx leftElevatorMotor = null;
    @Getter
    private DcMotorEx rightElevatorMotor = null;
    @Getter
    private DcMotorEx intake = null;
    @Getter
    private Servo clawServo = null;
    @Getter
    private Servo rightArmServo = null;
    @Getter
    private Servo leftArmServo = null;
    @Getter
    private Servo planeLauncherServo = null;
    @Getter
    private Rev2mDistanceSensor right2mSensor = null;
    @Getter
    private Rev2mDistanceSensor left2mSensor = null;
    @Getter
    private Rev2mDistanceSensor back2mSensor = null;
    @Getter
    private BHI260IMU imu = null;
    @Getter
    private WebcamName webcam = null;
    @Getter
    private Elevator elevatorController = null;
    @Getter
    private Claws clawsController = null;

    @Getter
    private final List<DcMotorEx> drivetrainMotors = new ArrayList<>();
    @Getter
    private VoltageSensor batteryVoltageSensor = null;

    @Getter
    private MecanumDriveController mecanumDriveController = null;

    /**
     * Sets bulk caching to AUTO and saves the voltage sensor to the field.
     */
    public void initLynxModule() {
        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : opMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void initMecanumDriveController() {
        initDrivetrainMotors();
        initIMU();
        mecanumDriveController = new MecanumDriveController(this);
    }

    /**
     * This is the function that we initialize the Motors with
     **/
    public void initDrivetrainMotors() {
        initLynxModule();

        leftFrontMotor = opMode.hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackMotor = opMode.hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontMotor = opMode.hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackMotor = opMode.hardwareMap.get(DcMotorEx.class, "right_back");
        leftElevatorMotor = opMode.hardwareMap.get(DcMotorEx.class, "left_elevator");
        rightElevatorMotor = opMode.hardwareMap.get(DcMotorEx.class, "right_elevator");
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");

        drivetrainMotors.add(leftFrontMotor);
        drivetrainMotors.add(leftBackMotor);
        drivetrainMotors.add(rightBackMotor);
        drivetrainMotors.add(rightFrontMotor);

        for (DcMotorEx motor : drivetrainMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }


        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftElevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setAllMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setAllMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (DriveConstants.RUN_USING_ENCODER) setAllMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID);
        }

        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setAllBrake();
        leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorController = new ElevatorController(this);
        clawsController = new ClawsController(this);
    }

    public void setAllMode(DcMotor.RunMode mode) {
        drivetrainMotors.forEach(dcMotorEx -> dcMotorEx.setMode(mode));
    }

    public void setAllBrake() {
        drivetrainMotors.forEach(dcMotorEx -> dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void setAllPowers(double v, double v1, double v2, double v3) {
        leftFrontMotor.setPower(v);
        leftBackMotor.setPower(v1);
        rightBackMotor.setPower(v2);
        rightFrontMotor.setPower(v3);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        drivetrainMotors.forEach(dcMotorEx -> dcMotorEx.setPIDFCoefficients(runMode, compensatedCoefficients));
    }

    /**
     * This is the function that we initialize the Claws with
     */

    public void initClaws() {

        clawServo = opMode.hardwareMap.get(Servo.class, "claw");
        rightArmServo = opMode.hardwareMap.get(Servo.class, "right_arm");
        leftArmServo = opMode.hardwareMap.get(Servo.class, "left_arm");
        planeLauncherServo = opMode.hardwareMap.get(Servo.class, "plane");

        leftArmServo.setDirection(Servo.Direction.FORWARD);
        rightArmServo.setDirection(Servo.Direction.REVERSE);

        //Approx 36 - 130 deg

        planeLauncherServo.scaleRange(0,0.4);
        leftArmServo.scaleRange(0.39,0.6);
        rightArmServo.scaleRange(0.39,0.6);
        clawServo.scaleRange(0,0.02);
    }

    /**
     * This is the function that we initialize the Sensors with
     */
    public void initSensors() {
        left2mSensor = (Rev2mDistanceSensor) opMode.hardwareMap.get(DistanceSensor.class, "left_2m");
        right2mSensor = (Rev2mDistanceSensor) opMode.hardwareMap.get(DistanceSensor.class, "right_2m");
        back2mSensor = (Rev2mDistanceSensor) opMode.hardwareMap.get(DistanceSensor.class, "back_2m");
    }

    /**
     * Initializes the Inertial Measurement Unit found in the Rev Expansion Hub
     */
    public void initIMU() {
        imu = opMode.hardwareMap.get(BHI260IMU.class, "imu");
    }

    /**
     * Initialize all required hardware for the Autonomous Period
     */
    public void initAutonomous() {
        initMecanumDriveController();
        initClaws();
        initWebcam();
    }

    /**
     * Initialize all required hardware for the Tele Op Period
     */
    public void initTeleOp() {
        initDrivetrainMotors();
        initClaws();
    }

    public void initWebcam() {
        webcam = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
    }
}
