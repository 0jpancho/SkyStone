package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;

public class Intake implements Subsystem{

    private HardwareMap hardwareMap;
    private DcMotor left, right;
    private Servo pivotL, pivotR;

    private PivotState pivotState = PivotState.STOW;
    private PowerState powerState = PowerState.STOP;


    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public enum PowerState {
        INTAKE(1.0),
        SPIT_OUT(-0.6),
        STOP(0.0);

        private final double power;

        PowerState(double power){
            this.power = power;
        }
    }

    public enum PivotState {
        STOW(0.0),
        DEPLOY(1.0);

        private final double postion;

        PivotState(double position){
            this.postion = position;
        }
    }

    public void setPowerState(PowerState powerState){
        this.powerState = powerState;
    }

    public void setPivotState(PivotState pivotState){
        this.pivotState = pivotState;
    }

    @Override
    public void initHardware(){
        this.left = hardwareMap.get(DcMotor.class, HardwareKeys.LEFT_NAME);
        this.right = hardwareMap.get(DcMotor.class, HardwareKeys.RIGHT_NAME);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotL = hardwareMap.servo.get("pivotL");
        pivotR = hardwareMap.servo.get("pivotR");

        pivotR.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void periodic(){
        left.setPower(powerState.power);
        right.setPower(powerState.power);

        pivotL.setPosition(pivotState.postion);
        pivotR.setPosition(pivotState.postion);
    }
}