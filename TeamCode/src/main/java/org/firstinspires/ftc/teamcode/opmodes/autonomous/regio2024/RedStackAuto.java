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

    private Trajectory purplePixelLeft;
    private Trajectory purplePixelRight;
    private TrajectorySequence purplePixelCenter;
    private Trajectory backboardLeft;
    private Trajectory backboardRight;
    private TrajectorySequence backboardCenter;
    private Trajectory parkLeft;
    private Trajectory parkRight;
    private TrajectorySequence parkCenter;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    protected void initTrajectories() {
        robotHardware.getMecanumDriveController().setPoseEstimate(startPose);

        purplePixelCenter = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(startPose)
                .forward(3)
                .strafeLeft(5)
                .forward(13)
                .build();
        backboardCenter = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(purplePixelCenter.end())
                .back(5)
                .strafeLeft(15)
                .forward(7)
                .build();

        parkCenter = robotHardware.getMecanumDriveController()
                .trajectorySequenceBuilder(backboardCenter.end())
                .strafeRight(15)
                .back(10)
                .build();

    }

    @Override
    protected void run() {

        States state = States.START;


        while (opModeIsActive()) {
            if (elementZone == 2) {
                boolean ok = true;
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
        }

    }
}
