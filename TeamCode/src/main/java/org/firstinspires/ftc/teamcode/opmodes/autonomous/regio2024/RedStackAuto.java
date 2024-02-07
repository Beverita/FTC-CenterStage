package org.firstinspires.ftc.teamcode.opmodes.autonomous.regio2024;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;
import org.firstinspires.ftc.teamcode.control.TeleOpControl;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedStack", group = "Regio", preselectTeleOp = "Regio TeleOp")
public class RedStackAuto extends AutonomousControl {

    enum States {
        START,
        PURPLE_PIXEL_LEFT,
        PURPLE_PIXEL_RIGHT,
        PURPLE_PIXEL_CENTER,
        ROTATE,
        BACKBOARD_LEFT,
        BACKBOARD_RIGHT,
        BACKBOARD_CENTER,
        PARK_LEFT,
        PARK_RIGHT,
        PARK_CENTER,
        END
    }

    private final Pose2d startPose = new Pose2d(-10, -60.5, Math.toRadians(90));

    private TrajectorySequence purplePixelLeft;
    private TrajectorySequence purplePixelRight;
    private TrajectorySequence purplePixelCenter;
    private TrajectorySequence backboardLeft;
    private TrajectorySequence backboardRight;
    private TrajectorySequence backboardCenter;
    private TrajectorySequence parkLeft;
    private TrajectorySequence parkRight;
    private TrajectorySequence parkCenter;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    protected void initTrajectories() {
        robotHardware.getMecanumDriveController().setPoseEstimate(startPose);

        purplePixelCenter = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(startPose)
                .forward(3)
                .strafeRight(5)
                .forward(13)
                .build();

        purplePixelLeft = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(startPose)
                .strafeRight(5)
                .forward(10)
                .build();

        purplePixelRight = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(startPose)
                .strafeRight(5)
                .forward(10)
                .strafeLeft(10)
                .build();

        backboardCenter = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(purplePixelCenter.end())
                .back(5)
                .strafeRight(15)
                .forward(7)
                .build();

        backboardLeft = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(purplePixelLeft.end())
                .back(5)
                .strafeRight(10)
                .forward(5)
                .build();

        backboardRight = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(purplePixelRight.end())
                .back(2)
                .strafeRight(20)
                .forward(5)
                .build();

        parkCenter = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(backboardCenter.end())
                .strafeLeft(15)
                .back(10)
                .build();

        parkLeft = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(backboardLeft.end())
                .strafeLeft(10)
                .back(10)
                .build();

        parkRight = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(backboardRight.end())
                .strafeLeft(20)
                .back(10)
                .build();

    }

    @Override
    protected void run() {

        States state = States.START;

        while (opModeIsActive()) {
            if(elementZone == 1) {
                switch (state) {
                    case START:
                        robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(purplePixelLeft);
                        state = States.PURPLE_PIXEL_LEFT;
                        runtime.reset();
                        break;

                    case PURPLE_PIXEL_LEFT:
                        if (runtime.milliseconds() > 3500) {
                            robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(backboardLeft);
                            state = States.ROTATE;
                            runtime.reset();
                        }
                        break;

                    case ROTATE:
                        if (runtime.milliseconds() > 5000) {
                            robotHardware.getMecanumDriveController().turnAsync(Math.toRadians(45));
                            state = States.PARK_LEFT;
                            runtime.reset();
                        }

                    case PARK_LEFT:
                        if (runtime.milliseconds() > 50000) {
                            robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(parkLeft);
                            state = States.END;
                        }
                        break;
                }

                robotHardware.getElevatorController().update();
                robotHardware.getMecanumDriveController().update();
            }

            if (elementZone == 2) {
                switch (state) {
                    case START:
                        robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(purplePixelCenter);
                        state = States.PURPLE_PIXEL_CENTER;
                        runtime.reset();
                        break;

                    case PURPLE_PIXEL_CENTER:
                        if (runtime.milliseconds() > 3500) {
                            robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(backboardCenter);
                            state = States.ROTATE;
                            runtime.reset();
                        }
                        break;

                    case ROTATE:
                        if(runtime.milliseconds()>5000){
                            robotHardware.getMecanumDriveController().turnAsync(Math.toRadians(45));
                            state = States.PARK_CENTER;
                            runtime.reset();
                        }

                    case PARK_CENTER:
                        if (runtime.milliseconds() > 50000) {
                            robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(parkCenter);
                            state = States.END;
                        }
                        break;
                }

                robotHardware.getElevatorController().update();
                robotHardware.getMecanumDriveController().update();
            }

            if (elementZone == 3) {
                switch (state) {
                    case START:
                        robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(purplePixelRight);
                        state = States.PURPLE_PIXEL_RIGHT;
                        runtime.reset();
                        break;

                    case PURPLE_PIXEL_RIGHT:
                        if (runtime.milliseconds() > 3500) {
                            robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(backboardRight);
                            state = States.ROTATE;
                            runtime.reset();
                        }
                        break;

                    case ROTATE:
                        if(runtime.milliseconds()>5000){
                            robotHardware.getMecanumDriveController().turnAsync(Math.toRadians(45));
                            state = States.PARK_RIGHT;
                            runtime.reset();
                        }

                    case PARK_RIGHT:
                        if (runtime.milliseconds() > 50000) {
                            robotHardware.getMecanumDriveController().followTrajectorySequenceAsync(parkRight);
                            state = States.END;
                        }
                        break;
                }

                robotHardware.getElevatorController().update();
                robotHardware.getMecanumDriveController().update();
            }

        }

    }
}
