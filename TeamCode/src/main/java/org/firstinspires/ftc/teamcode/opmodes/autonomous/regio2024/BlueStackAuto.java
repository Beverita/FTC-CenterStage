package org.firstinspires.ftc.teamcode.opmodes.autonomous.regio2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;

@Autonomous(name = "BlueBackboard", group = "Regio",preselectTeleOp = "Regio TeleOp")
public class BlueStackAuto extends AutonomousControl {


    private  enum States{
        START,PUSHPURPLEPIXEL,GOTOBACKBOARD,PARK
    }
    private final Pose2d startPose = new Pose2d(35,-60.5,Math.toRadians(270));

    private Trajectory pushPurplePixel;

    private Trajectory goToBackboard;

    private Trajectory park;

    @Override
    protected void initTrajectories() {
        robotHardware.getMecanumDriveController().setPoseEstimate(startPose);
    }

    @Override
    protected void run() {
        States state = States.START;

        while(opModeIsActive()){
            switch(state){
                case START:
                    robotHardware.getMecanumDriveController().followTrajectoryAsync(pushPurplePixel);
                    state=States.PUSHPURPLEPIXEL;
                    break;
                case PUSHPURPLEPIXEL:
                    robotHardware.getMecanumDriveController().followTrajectoryAsync(goToBackboard);
                    state=States.GOTOBACKBOARD;
                    break;
                case GOTOBACKBOARD:
                    robotHardware.getMecanumDriveController().followTrajectoryAsync(park);
                    state =States.PARK;
                    break;
                }
            robotHardware.getElevatorController().update();
            robotHardware.getMecanumDriveController().update();
        }
    }
}
