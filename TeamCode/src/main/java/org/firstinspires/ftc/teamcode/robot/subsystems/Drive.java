package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;
import org.firstinspires.ftc.teamcode.util.MotorPair;

public class Drive implements Subsystem{
    private HardwareMap hardwareMap;

    public DcMotorEx frontLeft, backLeft, frontRight, backRight;
    public MotorPair leftStraightPair, rightStraightPair, leftStrafePair, rightStrafePair;

    private double frontLeftPow = 0;
    private double backLeftPow = 0;
    private double frontRightPow = 0;
    private double backRightPow = 0;

    public Drive(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void setPower(double frontLeftPow, double backLeftPow,double frontRightPow, double backRightPow){
        this.frontLeftPow = frontLeftPow;
        this.backLeftPow = backLeftPow;
        this.frontRightPow = frontRightPow;
        this.backRightPow = backRightPow;
    }

    @Override
    public void initHardware(){
        frontLeft = hardwareMap.get(DcMotorEx.class, HardwareKeys.FL_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, HardwareKeys.BL_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, HardwareKeys.FR_NAME);
        backRight = hardwareMap.get(DcMotorEx.class,HardwareKeys.BR_NAME);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftStraightPair = new MotorPair(frontLeft, backLeft);
        rightStraightPair = new MotorPair(frontRight, backRight);

        leftStrafePair = new MotorPair(frontLeft, backRight);
        rightStrafePair = new MotorPair(frontRight, backLeft);

        brakeMode(true);
    }

    @Override
    public void periodic(){
        frontLeft.setPower(frontLeftPow);
        backLeft.setPower(backLeftPow);
        frontRight.setPower(frontRightPow);
        backRight.setPower(backRightPow);
    }

    public void setPIDFCoeffs(DcMotor.RunMode runMode, PIDFCoefficients coefficients){

        frontLeft.setPIDFCoefficients(runMode, coefficients);
        backLeft.setPIDFCoefficients(runMode, coefficients);

        frontRight.setPIDFCoefficients(runMode, coefficients);
        backRight.setPIDFCoefficients(runMode, coefficients);
    }

    public void brakeMode(boolean isBreak){
        if(isBreak){
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        else{
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setRunMode(DcMotor.RunMode runMode){

        frontLeft.setMode(runMode);
        backLeft.setMode(runMode);

        frontRight.setMode(runMode);
        backRight.setMode(runMode);
    }

    void setStrL(double power){
        leftStraightPair.setPower(power);
    }

    void setStrR(double power) {
        rightStraightPair.setPower(power);
    }

    public void setStrDrive(double left, double right) {
        setStrL(left);
        setStrR(right);
    }

    public void stopStrMotors() {
        leftStraightPair.stop();
        rightStraightPair.stop();
    }

    void setStrafeL(double power){
        leftStrafePair.setPower(power);
    }

    void setStrafeR(double power){
        rightStrafePair.setPower(power);
    }

    public void setStrafeDrive(double left, double right){
        setStrafeL(left);
        setStrafeR(right);
    }

    public void stopStrafeMotors(){
        leftStrafePair.stop();
        rightStrafePair.stop();
    }
}