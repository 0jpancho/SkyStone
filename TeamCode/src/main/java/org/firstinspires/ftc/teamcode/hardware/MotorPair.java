package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorPair {

    public DcMotorEx motorOne, motorTwo;
    public double power;
    public double position;
    public int encOffset = 0;

    public MotorPair(DcMotorEx motorOne, DcMotorEx motorTwo) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
    }

    public void setPower(double power) {
        this.power = power;
        motorOne.setPower(power);
        motorTwo.setPower(power);
    }

    public void setTargetPosition(int position) {
        this.position = position;
        motorOne.setTargetPosition(position);
        motorTwo.setTargetPosition(position);
    }

    public void stop() {
        setPower(0);
    }

    public double getPower() {
        return this.power;
    }

    public int getEncoder() {
        return (motorOne.getCurrentPosition() + motorTwo.getCurrentPosition()) / 2 - encOffset;
    }

    public int resetEncoder() {
        this.encOffset = (motorOne.getCurrentPosition() + motorTwo.getCurrentPosition()) / 2;
        return this.encOffset;
    }

    public void resetEncoder(int offset) {
        this.encOffset = offset;
    }

    public void floatMode() {
        this.motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void brakeMode() {
        this.motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMode(DcMotor.RunMode mode) {
        this.motorOne.setMode(mode);
        this.motorTwo.setMode(mode);
    }
}
