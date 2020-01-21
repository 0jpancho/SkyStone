package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;

public class Lift implements Subsystem {

    private HardwareMap hardwareMap;
    private DcMotor lift;
    private double liftPower;

    public Lift(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initHardware(){
        lift = hardwareMap.dcMotor.get(HardwareKeys.LIFT_NAME);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic(){
        lift.setPower(liftPower);
    }

    public void setLiftPower(double power){
        this.liftPower = power;

    }
}
