package org.firstinspires.ftc.teamcode.control.limbs;

import org.firstinspires.ftc.teamcode.control.Claws;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.constants.DriveConstants;

public class ClawsController implements Claws {
    private final RobotHardware robotHardware;
    private boolean clawState = false;
    private boolean armState = false;
    private boolean planeLauncherState = false;

    public ClawsController(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    @Override
    public boolean useClaws() {
        if (clawState) {
            robotHardware.getClawServo().setPosition(DriveConstants.CLAW_CLOSE_POSITION);
            clawState = false;
        } else {
            robotHardware.getClawServo().setPosition(DriveConstants.CLAW_OPEN_POSITION);
            clawState = true;
        }

        return clawState;
    }

    @Override
    public void useClaws(boolean state) {
        if (state) {
            robotHardware.getClawServo().setPosition(DriveConstants.CLAW_OPEN_POSITION);
            clawState = true;
        } else {
            robotHardware.getClawServo().setPosition(DriveConstants.CLAW_CLOSE_POSITION);
            clawState = false;
        }
    }

    @Override
    public boolean useArm(){
        if(armState){
            robotHardware.getLeftArmServo().setPosition(DriveConstants.ARM_UP_POSITION);
            robotHardware.getRightArmServo().setPosition(DriveConstants.ARM_UP_POSITION);
            armState = false;
        } else {
            robotHardware.getLeftArmServo().setPosition(DriveConstants.ARM_DOWN_POSITION);
            robotHardware.getRightArmServo().setPosition(DriveConstants.ARM_DOWN_POSITION);
            armState = true;
        }
        return armState;
    }

    @Override
    public void useArm(boolean state){
        if (state) {
            robotHardware.getLeftArmServo().setPosition(DriveConstants.ARM_UP_POSITION);
            robotHardware.getRightArmServo().setPosition(DriveConstants.ARM_UP_POSITION);
            armState = false;
        } else {
            robotHardware.getLeftArmServo().setPosition(DriveConstants.ARM_DOWN_POSITION);
            robotHardware.getRightArmServo().setPosition(DriveConstants.ARM_DOWN_POSITION);
            armState = true;
        }
    }

    @Override
    public void usePlaneLauncher(){
        if(planeLauncherState){
            robotHardware.getPlaneLauncherServo().setPosition(DriveConstants.PLANE_LAUNCH_POSITION);
            planeLauncherState = false;
        }else{
            robotHardware.getPlaneLauncherServo().setPosition(DriveConstants.PLANE_INIT_POSITION);
            planeLauncherState = true;
        }
    }

    @Override
    public boolean isClaws() {
        return clawState;
    }
}
