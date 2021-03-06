package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareKeys;

public class Intake implements Subsystem{

    private HardwareMap hardwareMap;
    private DcMotor left, right;
    private Servo pivotL, pivotR;

    private PivotState pivotState = PivotState.STOW;
    private PowerState powerState = PowerState.STOP;

    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    //Intake motor power states
    public enum PowerState {
        INTAKE(1.0),
        SPIT_OUT(-0.6),
        STOP(0.0);

        private final double power;

        PowerState(double power){
            this.power = power;
        }
    }

    //Intake drop servo pivot states
    public enum PivotState {
        STOW(1.0),
        DEPLOY(0);

        private final double position;

        PivotState(double position){
            this.position = position;
        }
    }

    //State changer for intake motors
    public void setPowerState(PowerState powerState){
        this.powerState = powerState;
    }

    //State changer for pivot servos
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

        pivotL = hardwareMap.servo.get(HardwareKeys.INTAKE_PIVOT_L_NAME);
        pivotR = hardwareMap.servo.get(HardwareKeys.INTAKE_PIVOT_R_NAME);

        pivotR.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void periodic(){
        left.setPower(powerState.power);
        right.setPower(powerState.power);

        pivotL.setPosition(pivotState.position);
        pivotR.setPosition(pivotState.position);
    }
}