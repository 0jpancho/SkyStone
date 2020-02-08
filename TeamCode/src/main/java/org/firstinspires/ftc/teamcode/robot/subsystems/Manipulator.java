package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareKeys;

public class Manipulator implements Subsystem {

    private HardwareMap hardwareMap;

    private Servo pivotL, pivotR, inverter, gripper;

    public PivotState pivotState = PivotState.STOW;
    public InverterState inverterState = InverterState.STOW;
    public GripperState gripperState = GripperState.RELEASE;

    public Manipulator(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    //Pivot servo states
    public enum PivotState {
        STOW(0),
        DEPLOY(1);

        private final double position;

        PivotState(double position){
            this.position = position;
        }
    }

    //Inverter servo states
    public enum InverterState {
        STOW(1),
        FLIP(0);

        private final double position;

        InverterState(double position){
            this.position = position;
        }
    }

    //Gripper servo states
    public enum GripperState{
        GRIP(0.5),
        RELEASE(0.0);

        private final double position;

        GripperState(double position){
            this.position = position;
        }
    }

    //State changer for pivot servos
    public void setPivotState(PivotState pivotState){
        this.pivotState = pivotState;
    }

    //State changer for inverter servo
    public void setInverterState(InverterState inverterState){
        this.inverterState = inverterState;
    }

    //State changer for gripper servo
    public void setGripperState(GripperState gripperState){
        this.gripperState = gripperState;
    }

    @Override
    public void initHardware(){
        pivotL = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_L);
        pivotR = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_R);

        inverter = hardwareMap.servo.get(HardwareKeys.INVERTER);
        gripper = hardwareMap.servo.get(HardwareKeys.GRIPPER_ARM);

        pivotR.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void periodic(){
        pivotL.setPosition(pivotState.position);
        pivotR.setPosition(pivotState.position);

        inverter.setPosition(inverterState.position);
        gripper.setPosition(gripperState.position);
    }
}
