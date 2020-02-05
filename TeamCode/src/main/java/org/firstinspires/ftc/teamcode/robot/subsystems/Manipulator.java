package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Manipulator implements Subsystem {

    private HardwareMap hardwareMap;

    //private CRServo pivotL, pivotR;

    //private double pivotPower;
    //private double multiplier;

    private Servo pivotL, pivotR, inverter, gripper;

    public PivotState pivotState = PivotState.STOW;
    public InverterState inverterState = InverterState.STOW;
    public GripperState gripperState = GripperState.RELEASE;

    public Manipulator(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }


    public enum PivotState {
        STOW(1),
        DEPLOY(0);

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

    /*
    public void setPivotPower(double power, double multiplier){

        this.pivotPower = power;
        this.multiplier = multiplier;

        pivotL.setPower(this.pivotPower * this.multiplier);
        pivotR.setPower(this.pivotPower * this.multiplier);
    }
    */


    @Override
    public void initHardware(){
        pivotL = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_L);
        pivotR = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_R);

        inverter = hardwareMap.servo.get(HardwareKeys.INVERTER);
        gripper = hardwareMap.servo.get(HardwareKeys.GRIPPER_ARM);

        //pivotL = hardwareMap.crservo.get(HardwareKeys.GRIPPER_PIVOT_L);
        //pivotR = hardwareMap.crservo.get(HardwareKeys.GRIPPER_PIVOT_R);

        pivotR.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void periodic(){
        pivotL.setPosition(pivotState.position);
        pivotR.setPosition(pivotState.position);

        inverter.setPosition(inverterState.position);
        gripper.setPosition(gripperState.position);

        //setPivotPower(pivotPower, multiplier);

    }
}
