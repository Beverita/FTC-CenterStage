package org.firstinspires.ftc.teamcode.control.limbs;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.Elevator;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.constants.DriveConstants;

import lombok.Getter;

public class ElevatorController implements Elevator {
    private final RobotHardware robotHardware;
    private int target = 0;
    @Getter
    private final PIDController pidController;

    private boolean bypass = false;

    public ElevatorController(RobotHardware robotHardware) {
        this(robotHardware, 150);
    }

    public ElevatorController(RobotHardware robotHardware, int tolerance) {
        this.robotHardware = robotHardware;
        pidController = new PIDController(8, 8.5, 1.3);
        pidController.setTolerance(tolerance);
    }

    @Override
    public void update() {
        //transform velocity to ratio
        double calculatedPower = pidController.calculate(robotHardware.getLeftElevatorMotor().getCurrentPosition(), target) / DriveConstants.ELEVATOR_MAX_TICKS_PER_SECOND;

        robotHardware.getRightElevatorMotor().setPower(calculatedPower);
        robotHardware.getLeftElevatorMotor().setPower(-calculatedPower);
    }

    @Override
    public void setTarget(int target) {
        if (!isBypass()) {
            target = (int) Range.clip(target, DriveConstants.elevatorCmToTicks(ElevatorLevel.BASE.getHeight()), DriveConstants.elevatorCmToTicks(ElevatorLevel.MAX.getHeight()));
        }

        this.target = target;
    }

    @Override
    public void setTarget(ElevatorLevel level) {
        this.target = (int) DriveConstants.elevatorCmToTicks(level.getHeight());
    }

    @Override
    public int getTarget() {
        return target;
    }

    @Override
    public int getCurrentPosition() {
        return robotHardware.getLeftElevatorMotor().getCurrentPosition();
    }

    @Override
    public boolean isBypass() {
        return bypass;
    }

    @Override
    public void setBypass(boolean bypass) {
        this.bypass = bypass;
    }
}
