package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.hardware.IMU;
import org.firstinspires.ftc.teamcode.util.MotorPair;
import org.firstinspires.ftc.teamcode.util.SynchronousPID;

import java.util.Arrays;

public class DriveTrain {

    public DcMotorEx frontLeft, backLeft, frontRight, backRight;

    LinearOpMode l;
    Telemetry t;

    MotorPair leftStraightPair, rightStraightPair, leftStrafePair, rightStrafePair;

    public DriveTrain(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry) {
        l = opMode;
        t = telemetry;

        frontLeft = hardwareMap.get(DcMotorEx.class, HardwareKeys.FL_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, HardwareKeys.BL_NAME);

        frontRight = hardwareMap.get(DcMotorEx.class, HardwareKeys.FR_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, HardwareKeys.BR_NAME);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        brakeMode(true);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftStraightPair = new MotorPair(frontLeft, backLeft);
        rightStraightPair = new MotorPair(frontRight, backRight);

        leftStrafePair = new MotorPair(frontLeft, backRight);
        rightStrafePair = new MotorPair(frontRight, backLeft);
    }

    public void mecanumArcadeDrive(double y, double x, double c){

        double frontLeftPower = y + x + c;
        double backLeftPower =  y - x + c;
        double frontRightPower = y - x - c;
        double backRightPower = y + x - c;

        double[] wheelPowers = {frontLeftPower, backLeftPower, frontRightPower, backRightPower};

        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            frontLeftPower /= wheelPowers[3];
            backLeftPower /= wheelPowers[3];
            frontRightPower /= wheelPowers[3];
            backRightPower /= wheelPowers[3];
        }

        frontLeft.setPower(-frontLeftPower);
        backLeft.setPower(-backLeftPower);

        frontRight.setPower(-frontRightPower);
        backRight.setPower(-backRightPower);

        t.addData("Speed Value", frontLeft.getPower());
    }

    public void mecanumRampTest(double y, double x, double c, double increment){

        if (y != 0){
            y += Math.abs(y) * increment;
        }

        if (x != 0){
            x += Math.abs(x) * increment;
        }

        if (c != 0){
            c += Math.abs(c) * increment;
        }

        double frontLeftPower = y - x - c;
        double backLeftPower =  y + x + c;
        double frontRightPower = y + x - c;
        double backRightPower = y - x + c;

        double[] wheelPowers = {frontLeftPower, backLeftPower, frontRightPower, backRightPower};

        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            frontLeftPower /= wheelPowers[3];
            backLeftPower /= wheelPowers[3];
            frontRightPower /= wheelPowers[3];
            backRightPower /= wheelPowers[3];
        }

        frontLeft.setPower(-frontLeftPower);
        backLeft.setPower(-backLeftPower);

        frontRight.setPower(-frontRightPower);
        backRight.setPower(-backRightPower);

        t.addData("Speed Value", frontLeft.getPower());
    }

    public void fieldCentricDrive(double y, double x, double rot, IMU imu){

        setPIDCoeffs(DcMotor.RunMode.RUN_USING_ENCODER, Constants.teleopDrive);

        double rawFLPow;
        double rawBLPow;
        double rawFRPow;
        double rawBRPow;

        double FLPow;
        double BLPow;
        double FRPow;
        double BRPow;

        double forward;
        double strafe;

        double degrees = imu.getHeading();
        double rads = degrees * (Constants.pi * 180);

        forward = y * Math.cos(rads) + x * Math.sin(rads);
        strafe = -y * Math.sin(rads) + x * Math.cos(rads);

        rawFLPow = forward - strafe - rot;
        rawBLPow = forward + strafe + rot;
        rawFRPow = forward + strafe - rot;
        rawBRPow = forward - strafe + rot;

        FLPow = Range.clip(rawFLPow, -1, 1);
        BLPow = Range.clip(rawBLPow, -1, 1);
        FRPow = Range.clip(rawFRPow, -1, 1);
        BRPow = Range.clip(rawBRPow, -1, 1);

        frontLeft.setPower(FLPow);
        backLeft.setPower(BLPow);
        frontRight.setPower(FRPow);
        backRight.setPower(BRPow);
    }


    public void correctStrafeDrive(double x, double toleranceDegrees, IMU imu, boolean strafeToggle){
        strafeToggle = false;
        imu.zeroHeading();
        double correctionP = 1;
        double initialHeading = imu.getHeading();
        double error = initialHeading - imu.getHeading();

        l.telemetry.addData("Initial Heading", initialHeading);
        l.telemetry.addData("Error", error);

        double speedCorrect = correctionP * error;

        while (l.opModeIsActive() && strafeToggle){
            if(x > 0 && strafeToggle){

                if(error > initialHeading){
                    leftStraightPair.setPower(-x * speedCorrect);
                    rightStraightPair.setPower(x * speedCorrect);
                }

                else if (error < initialHeading){
                    leftStraightPair.setPower(x * speedCorrect);
                    rightStraightPair.setPower(-x * speedCorrect);
                }

                else if (Math.abs(error - initialHeading) < toleranceDegrees){
                    leftStrafePair.setPower(x * speedCorrect);
                    rightStrafePair.setPower(-x * speedCorrect);
                }
            }
            else if (x < 0 && strafeToggle) {

                if(error > initialHeading){
                    leftStraightPair.setPower(x * speedCorrect);
                    rightStraightPair.setPower(-x * speedCorrect);
                }

                else if (error < initialHeading){
                    leftStraightPair.setPower(-x * speedCorrect);
                    rightStraightPair.setPower(x * speedCorrect);
                }

                else if (Math.abs(error - initialHeading) < toleranceDegrees){
                    leftStrafePair.setPower(-x * speedCorrect);
                    rightStrafePair.setPower(x * speedCorrect);
                }
            }
        }
        stopStrMotors();
    }

    public void driveForwardsBackwards(double maxSpeed, double p, double i, double d, double inches, long delay) {

        setPIDCoeffs(DcMotor.RunMode.RUN_USING_ENCODER, Constants.autoDrive);

        leftStraightPair.resetEncoder();
        rightStraightPair.resetEncoder();

        double targetPulses = inches * Constants.countsPerInch();

        SynchronousPID leftPID = new SynchronousPID(p, i, d);
        SynchronousPID rightPID = new SynchronousPID(p, i, d);

        leftPID.setOutputRange(-maxSpeed, maxSpeed);
        leftPID.setSetpoint(targetPulses);

        rightPID.setOutputRange(-maxSpeed, maxSpeed);
        rightPID.setSetpoint(targetPulses);

        int leftEncoder = this.leftStraightPair.getEncoder();
        int rightEncoder = this.rightStraightPair.getEncoder();

        double left = leftPID.calculate(leftEncoder);
        double right = rightPID.calculate(rightEncoder);

        l.sleep(delay * 1000);

        while (!(Math.abs(leftEncoder - targetPulses) < 25) || !(Math.abs(rightEncoder - targetPulses) < 25)
                && l.opModeIsActive() && l.isStarted() && !l.isStopRequested()){

            t.addData("Left PID State", leftPID.getState());
            t.addData("Left Setpoint", leftPID.getState());

            t.addData("Right PID State", rightPID.getState());
            t.addData("Right PID State", rightPID.getState());

            setStrDrive(left, right);
        }
        stopStrMotors();
    }

    public void strafePID(double maxSpeed, double p, double i, double d, double inches, long delay, boolean isLeft){

        setPIDCoeffs(DcMotor.RunMode.RUN_USING_ENCODER, Constants.autoDrive);

        leftStraightPair.resetEncoder();
        rightStraightPair.resetEncoder();

        double targetPulses = inches * Constants.countsPerInch();

        SynchronousPID leftPID = new SynchronousPID(p, i, d);
        SynchronousPID rightPID = new SynchronousPID(p, i, d);

        leftPID.setOutputRange(-maxSpeed, maxSpeed);
        leftPID.setSetpoint(targetPulses);

        rightPID.setOutputRange(-maxSpeed, maxSpeed);
        rightPID.setSetpoint(targetPulses);

        int leftEncoder = this.leftStraightPair.getEncoder();
        int rightEncoder = this.rightStraightPair.getEncoder();

        double left = leftPID.calculate(leftEncoder);
        double right = rightPID.calculate(rightEncoder);

        if(isLeft){
            while (!(Math.abs(leftEncoder - targetPulses) < 25) || !(Math.abs(rightEncoder - targetPulses) < 25)
                    && l.opModeIsActive() && l.isStarted() && !l.isStopRequested()) {

                t.addData("Left PID State", leftPID.getState());
                t.addData("Left Setpoint", leftPID.getSetpoint());

                t.addData("Right PID State", rightPID.getState());
                t.addData("Right PID State", rightPID.getSetpoint());

                setStrafeDrive(-left, right);

                t.update();
            }
        }

        else {
            while (!(Math.abs(leftEncoder - targetPulses) < 25) || !(Math.abs(rightEncoder - targetPulses) < 25)
                    && l.opModeIsActive() && l.isStarted() && !l.isStopRequested()) {

                t.addData("Left PID State", leftPID.getState());
                t.addData("Left Setpoint", leftPID.getState());

                t.addData("Right PID State", rightPID.getState());
                t.addData("Right PID State", rightPID.getState());

                setStrafeDrive(left, -right);

                t.update();
            }
        }
        stopStrafeMotors();
    }

    public void driveGyroCorrection(double maxSpeed, double p, double i, double d, IMU imu, double inches, long delay) {

        imu.zeroHeading();
        double correctionP = 1;
        double initialHeading = imu.getHeading();
        double error = initialHeading - imu.getHeading();

        double speedCorrect = correctionP * error;

        leftStraightPair.resetEncoder();
        rightStraightPair.resetEncoder();

        double targetPulses = inches * Constants.countsPerInch();

        SynchronousPID leftPID = new SynchronousPID(p, i, d);
        SynchronousPID rightPID = new SynchronousPID(p, i, d);

        leftPID.setOutputRange(-maxSpeed, maxSpeed);
        leftPID.setSetpoint(targetPulses);

        rightPID.setOutputRange(-maxSpeed, maxSpeed);
        rightPID.setSetpoint(targetPulses);

        int leftEncoder = this.leftStraightPair.getEncoder();
        int rightEncoder = this.rightStraightPair.getEncoder();

        double left = leftPID.calculate(leftEncoder);
        double right = rightPID.calculate(rightEncoder);

        l.sleep(delay * 1000);

        while(!(Math.abs(leftEncoder - targetPulses) < 25) || !(Math.abs(rightEncoder - targetPulses) < 25)
                && l.opModeIsActive() && l.isStarted() && !l.isStopRequested()){

            t.addData("Left PID State", leftPID.getState());
            t.addData("Left Setpoint", leftPID.getState());

            t.addData("Right PID State", rightPID.getState());
            t.addData("Right Setpoint", rightPID.getState());

            if(error > initialHeading){
                setStrDrive(-left * speedCorrect, right * speedCorrect);
            }
            else if (error < initialHeading){
                setStrDrive(left * speedCorrect, -right * speedCorrect);
            }
            else{
                setStrDrive(left, right);
            }
            t.update();
        }
        stopStrMotors();
    }

    public void turnPID(double maxTurn, double kp, double ki, double kd, double angle, IMU imu,
                        double turnTolerance, long delay) {

        setPIDCoeffs(DcMotor.RunMode.RUN_USING_ENCODER, Constants.autoTurn);

        SynchronousPID turnPID = new SynchronousPID(kp, ki, kd);

        turnPID.setOutputRange(-maxTurn, maxTurn);
        turnPID.setSetpoint(angle);
        turnPID.setDeadband(turnTolerance);

        l.sleep(delay * 1000);

        double heading = imu.getHeading();
        double turnFactor = turnPID.calculateGivenError(angle - imu.getHeading());

        if (turnFactor > -.07 && turnFactor < 0) {
            turnFactor = -.07;
        }
        if (turnFactor < .07 && turnFactor > 0) {
            turnFactor = .07;
        }

        while(!(Math.abs(turnPID.getError()) < turnTolerance)
                && l.opModeIsActive() && l.isStarted() && !l.isStopRequested()){
            setStrDrive(turnFactor, -turnFactor);

            t.addData("heading", heading);
            t.addData("turn error", turnPID.getError());
            t.addData("turnFactor", turnFactor);
            t.update();
        }
        stopStrMotors();
    }

    void setStrL(double power){
        leftStraightPair.setPower(power);
    }

    void setStrR(double power) {
        rightStraightPair.setPower(power);
    }

    void setStrDrive(double left, double right) {
        setStrL(left);
        setStrR(right);
    }

    void stopStrMotors() {
        leftStraightPair.stop();
        rightStraightPair.stop();
    }

    void setStrafeL(double power){
        leftStrafePair.setPower(power);
    }

    void setStrafeR(double power){
        rightStrafePair.setPower(power);
    }

    void setStrafeDrive(double left, double right){
        setStrafeL(left);
        setStrafeR(right);
    }

    void stopStrafeMotors(){
        leftStrafePair.stop();
        rightStrafePair.stop();
    }

    public void setPIDCoeffs(DcMotor.RunMode runMode, PIDFCoefficients coefficients){

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
}


