package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class IMU implements Subsystem {

    public BNO055IMU imu;
    public Orientation angles;

    private HardwareMap hardwareMap;
    private Telemetry t;

    private float headingOffset;


    public IMU(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        t = telemetry;
    }

    @Override
    public void initHardware() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "RevIMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        zeroHeading();
    }

    @Override
    public void periodic() {
        t.addData("IMU Calibrated?", isCalibrated());
        t.addData("Normalized Heading", getHeading());
        t.addData("Raw Z Axis", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
        t.update();
    }

    public void zeroHeading() {
        this.headingOffset = imu.getAngularOrientation().firstAngle * -1;
    }

    public double getHeading() {
        return normalize360(imu.getAngularOrientation().firstAngle * -1 - headingOffset);
    }

    public static double normalize360(double angle) {

        /*while(angle >= 360) {
            angle -= 360;
        }
        while(angle < 0) {
            angle += 360;
        }
        */
        return angle;
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

    public boolean isCalibrated(){
        return imu.isGyroCalibrated();
    }

}
