package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    private final RobotHardware robotHardware = new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.initMecanumDriveController();

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = robotHardware.getMecanumDriveController().trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        robotHardware.getMecanumDriveController().followTrajectory(traj);

        sleep(2000);

        robotHardware.getMecanumDriveController().followTrajectory(
                robotHardware.getMecanumDriveController().trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}
