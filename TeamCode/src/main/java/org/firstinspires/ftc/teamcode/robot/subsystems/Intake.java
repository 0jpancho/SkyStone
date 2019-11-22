package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;

public class Intake implements Subsystem{

    private HardwareMap hardwareMap;
    private DcMotor left, right, pivot;

    private double pivotPower = 0;
    private State state = State.STOP;

    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public enum State{
        INTAKE(1.0),
        SPIT_OUT(-0.6),
        STOP(0.0);

        private final double power;

        State(double power){
            this.power = power;
        }
    }

    public void setState(State state){
        this.state = state;
    }

    public void setPivotPow(double pivotPower){
        this.pivotPower = pivotPower;
    }

    @Override
    public void initHardware(){
        this.left = hardwareMap.get(DcMotor.class, HardwareKeys.LEFT_NAME);
        this.right = hardwareMap.get(DcMotor.class, HardwareKeys.RIGHT_NAME);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        this.pivot = hardwareMap.get(DcMotorEx.class, HardwareKeys.PIVOT_NAME);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic(){
        left.setPower(state.power);
        right.setPower(state.power);

        pivot.setPower(pivotPower);
    }
}