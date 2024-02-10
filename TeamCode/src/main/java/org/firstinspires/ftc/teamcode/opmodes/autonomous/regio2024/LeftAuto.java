package org.firstinspires.ftc.teamcode.opmodes.autonomous.regio2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Left", group = "Regio", preselectTeleOp = "Regio TeleOp")
@Disabled
public class LeftAuto extends AutonomousControl {
    enum States {
        START,
        PARK,
        PUSH,
        END
    }

    private final Pose2d startPose = new Pose2d(-10, -60.5, Math.toRadians(90));

    private TrajectorySequence parkLeft;
    private TrajectorySequence parkRight;
    private Trajectory parkCenter;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    protected void initTrajectories() {
        robotHardware.getMecanumDriveController().setPoseEstimate(startPose);

        parkLeft = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(startPose)
                .forward(10)
                .strafeLeft(5)
                .build();

        parkRight = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(startPose)
                .forward(10)
                .strafeRight(5)
                .build();

        parkCenter = robotHardware.getMecanumDriveController()
                .trajectoryBuilder(startPose)
                .forward(15)
                .build();
    }

    @Override
    protected void run() {
        States state = States.START;

        while (opModeIsActive()) {
            if (elementZone == 1) {
                switch (state) {
                    case START:
                        robotHardware.getClawsController().useClaws();
                        robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(parkLeft);
                        state = States.PARK;
                        runtime.reset();
                        break;

                    case PARK:
                        if (runtime.milliseconds() > 3000) {
                            robotHardware.getIntake().setPower(-0.35);
                            state = States.PUSH;
                            runtime.reset();
                        }
                        break;

                    case PUSH:
                        if (runtime.milliseconds() > 1500) {
                            robotHardware.getIntake().setPower(0);
                            state = States.END;
                        }
                        break;
                }
            }

            if (elementZone == 2) {
                switch (state) {
                    case START:
                        robotHardware.getClawsController().useClaws();
                        robotHardware.getMecanumDriveController().followTrajectoryAsync(parkCenter);
                        state = States.PARK;
                        runtime.reset();
                        break;

                    case PARK:
                        if (runtime.milliseconds() > 3000) {
                            robotHardware.getIntake().setPower(-0.35);
                            state = States.PUSH;
                            runtime.reset();
                        }
                        break;

                    case PUSH:
                        if (runtime.milliseconds() > 1500) {
                            robotHardware.getIntake().setPower(0);
                            state = States.END;
                        }
                        break;
                }
            }

            if (elementZone == 3) {
                switch (state) {
                    case START:
                        robotHardware.getClawsController().useClaws();
                        robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(parkRight);
                        state = States.PARK;
                        runtime.reset();
                        break;

                    case PARK:
                        if (runtime.milliseconds() > 3000) {
                            robotHardware.getIntake().setPower(-0.35);
                            state = States.PUSH;
                            runtime.reset();
                        }
                        break;

                    case PUSH:
                        if (runtime.milliseconds() > 1500) {
                            robotHardware.getIntake().setPower(0);
                            state = States.END;
                        }
                        break;
                }
            }
        }


    }
}
