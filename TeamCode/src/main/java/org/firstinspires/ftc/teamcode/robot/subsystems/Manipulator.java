package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;

public class Manipulator implements Subsystem {

    private HardwareMap hardwareMap;

    private Servo pivotL, pivotR, inverter, gripper;

    public PivotState pivotState = PivotState.STOW;
    public InverterState inverterState = InverterState.STOW;
    public GripperState gripperState = GripperState.RELEASE;

    public Manipulator(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public enum PivotState {
        STOW(0.0),
        DEPLOY(1.0);

        private final double position;

        PivotState(double position){
            this.position = position;
        }
    }

    public enum InverterState {
        STOW(0.0),
        FLIP(1.0);

        private final double position;

        InverterState(double position){
            this.position = position;
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

    public void setPivotState(PivotState pivotState){
        this.pivotState = pivotState;
    }

    public void setInverterState(InverterState inverterState){
        this.inverterState = inverterState;
    }

    public void setGripperState(GripperState gripperState){
        this.gripperState = gripperState;
    }

    @Override
    public void initHardware(){
        pivotL = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_L);
        pivotR = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_R);

        pivotR.setDirection(Servo.Direction.REVERSE);

        inverter = hardwareMap.servo.get(HardwareKeys.INVERTER);
        gripper = hardwareMap.servo.get(HardwareKeys.GRIPPER_ARM);
    }

    @Override
    public void periodic(){
        pivotL.setPosition(pivotState.position);
        pivotR.setPosition(pivotState.position);

        inverter.setPosition(inverterState.position);
        gripper.setPosition(gripperState.position);

    }
}
