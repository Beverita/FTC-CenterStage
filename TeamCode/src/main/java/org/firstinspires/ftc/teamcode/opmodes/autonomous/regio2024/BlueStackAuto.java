package org.firstinspires.ftc.teamcode.opmodes.autonomous.regio2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;


@Autonomous(name = "BlueStack", group = "Regio",preselectTeleOp = "Regio TeleOp")
public class BlueStackAuto extends AutonomousControl{

    private enum States{

        START,PUSHPURPLEPIXEL,GOTOBACKBOARD,PARK
    }

    private final Pose2d startPose = new Pose2d(-35,-60.5,Math.toRadians(90));

    private Trajectory pushPurplePixel;

    private Trajectory goToBackboard;

    private Trajectory park;
    @Override
    protected void initTrajectories() {
        robotHardware.getMecanumDriveController().setPoseEstimate(startPose);

        pushPurplePixel = robotHardware.getMecanumDriveController()
                .trajectoryBuilder(startPose)
                .lineTo(new Vector2d(35,42.5))
                .build();

        goToBackboard=robotHardware.getMecanumDriveController()
                .trajectoryBuilder(pushPurplePixel.end())
                .strafeLeft(18)
                .splineToLinearHeading(new Pose2d(53,24.5),Math.toRadians(0))
                .lineTo(new Vector2d(-53,24.5))
                .lineTo(new Vector2d(-53,-42.5))
                .build();
    }

    @Override
    protected void run() {
        States state = States.START;

        while(opModeIsActive()){
            switch(state){
                case START:
                    robotHardware.getMecanumDriveController().followTrajectoryAsync(pushPurplePixel);
                    state = States.PUSHPURPLEPIXEL;
                    break;

                case PUSHPURPLEPIXEL:
                    robotHardware.getMecanumDriveController().followTrajectoryAsync(goToBackboard);
                    state = States.GOTOBACKBOARD;
                    break;
                case PARK:
                    robotHardware.getMecanumDriveController().followTrajectoryAsync(park);
                    state=States.PARK;
                    break;
            }
            robotHardware.getElevatorController().update();
            robotHardware.getMecanumDriveController().update();
        }
    }


}
