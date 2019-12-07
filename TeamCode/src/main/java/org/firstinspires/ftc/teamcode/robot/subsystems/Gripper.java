package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;

public class Gripper implements Subsystem {

    private HardwareMap hardwareMap;

    private Servo pivotL, pivotR, gripper;

    private PivotState pivotState;
    private GripperState gripperState;

    public Gripper(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public enum PivotState {
        STOW(0.0),
        DEPLOY(1.0);

        private final double postion;

        PivotState(double position){
            this.postion = position;
        }
    }

    public enum GripperState{
        GRIP(1.0),
        RELEASE(0.0);

        private final double position;

        GripperState(double position){
            this.position = position;
        }
    }

    public void setPivotState(PivotState state){
        this.pivotState = state;
    }

    public void setGripperState(GripperState state){
        this.gripperState = state;
    }

    @Override
    public void initHardware(){
        pivotL = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_L);
        pivotR = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_R);

        pivotR.setDirection(Servo.Direction.REVERSE);

        gripper = hardwareMap.servo.get(HardwareKeys.GRIPPER_ARM);
    }

    @Override
    public void periodic(){
        pivotL.setPosition(pivotState.postion);
        pivotR.setPosition(pivotState.postion);

        gripper.setPosition(gripperState.position);
    }
}
