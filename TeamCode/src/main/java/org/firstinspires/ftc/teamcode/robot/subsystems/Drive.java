package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareKeys;
import org.firstinspires.ftc.teamcode.util.MotorPair;

public class Drive implements Subsystem{
    private HardwareMap hardwareMap;
    private Telemetry t;

    public DcMotorEx frontLeft, backLeft, frontRight, backRight;
    public MotorPair leftStraightPair, rightStraightPair, leftStrafePair, rightStrafePair;

    private Servo leftMover, rightMover, stoneGrabber;

    private MoverState moverState = MoverState.STOW;
    private GrabberState grabberState = GrabberState.STOW;

    private ColorSensor colorSensor;

    private double frontLeftPow = 0;
    private double backLeftPow = 0;
    private double frontRightPow = 0;
    private double backRightPow = 0;

    public BNO055IMU imu;

    private float headingOffset;

    public enum MoverState {
        STOW(0.0),
        DEPLOY(0.5);

        private final double position;

        MoverState(double position){
            this.position = position;
        }
    }

    public enum GrabberState {
        STOW(0.4),
        DEPLOY(0.0);

        private final double position;

        GrabberState(double position){
            this.position = position;
        }
    }

    public Drive(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.t = telemetry;
    }

    public void setPower(double frontLeftPow, double backLeftPow, double frontRightPow, double backRightPow){
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

        leftMover = hardwareMap.servo.get(HardwareKeys.MOVER_L_NAME);
        rightMover = hardwareMap.servo.get(HardwareKeys.MOVER_R_NAME);
        stoneGrabber = hardwareMap.servo.get(HardwareKeys.STONE_GRABBER_NAME);

        colorSensor = hardwareMap.colorSensor.get(HardwareKeys.COLOR_SENSOR_NAME);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag = "RevIMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        zeroHeading();
    }

    @Override
    public void periodic(){
        frontLeft.setPower(frontLeftPow);
        backLeft.setPower(backLeftPow);
        frontRight.setPower(frontRightPow);
        backRight.setPower(backRightPow);

        leftMover.setPosition(moverState.position);
        rightMover.setPosition(moverState.position);
        stoneGrabber.setPosition(grabberState.position);

        t.addData("Red", colorSensor.red());
        t.addData("Green", colorSensor.green());
        t.addData("Blue", colorSensor.blue());
        t.addData("IMU Calibrated?", isCalibrated());
        t.addData("Normalized Heading", getHeading());
        t.addData("Raw ZYX Axis", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
        t.update();
    }

    public void setMoverState(MoverState state){
        this.moverState = state;
    }

    public void setGrabberState(GrabberState state){
        this.grabberState = state;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);

        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backRight .setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setRunMode(DcMotor.RunMode runMode){

        frontLeft.setMode(runMode);
        backLeft.setMode(runMode);

        frontRight.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void zeroHeading() {
        this.headingOffset = imu.getAngularOrientation().firstAngle * -1;
    }

    public double getHeading() {
        return normalize360(imu.getAngularOrientation().firstAngle * -1 - headingOffset);
    }

    public static double normalize180(double angle) {
        while(angle > 180) {
            angle -= 360;
        }
        while(angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    private static double normalize360(double angle) {
        /*
        while(angle >= 360) {
            angle -= 360;
        }
        while(angle < 0) {
            angle += 360;
        }
        */
        return angle;
    }

    private boolean isCalibrated(){
        return imu.isGyroCalibrated();
    }

    private int getR(){
        return colorSensor.red();
    }

    private int getG(){
        return colorSensor.green();
    }

    private int getB(){
        return colorSensor.blue();
    }

    public boolean stoneFound(){

        //Place holder value
        if (getR() > 5){
            return true;
        }

        else{
            return false;
        }
    }

    private void setStrL(double power){
        leftStraightPair.setPower(power);
    }

    private void setStrR(double power) {
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

    private void setStrafeL(double power){
        leftStrafePair.setPower(power);
    }

    private void setStrafeR(double power){
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
