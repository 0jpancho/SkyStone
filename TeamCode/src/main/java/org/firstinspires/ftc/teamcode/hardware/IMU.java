package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {

    Telemetry t;

    private BNO055IMU imu;
    private Orientation angles;

    float headingOffset;

    public IMU(HardwareMap hardwareMap, Telemetry telemetry){

        t = telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void zeroHeading() {
        this.headingOffset = imu.getAngularOrientation().firstAngle * -1;
    }

    public double getHeading() {
        return normalize360(imu.getAngularOrientation().firstAngle * -1 - headingOffset);
    }

    public static double normalize360(double angle) {
        while(angle >= 360) {
            angle -= 360;
        }
        while(angle < 0) {
            angle += 360;
        }
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

    public void imuTelemetry(){
        t.addData("Gyro Calibrating?", isCalibrated());
        t.addData("Normalized Heading", getHeading());
        t.addData("Raw Z Axis", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
        t.update();
    }

    /*
    byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
    byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

    //Need to be in CONFIG mode to write to registers
    imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.S ensorMode.CONFIG.bVal & 0x0F);

    sleep(100); //Changing modes requires a delay before doing anything else

    //Write to the AXIS_MAP_CONFIG register
    imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS _MAP_CONFIG_BYTE & 0x0F);

    //Write to the AXIS_MAP_SIGN register
    imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_M AP_SIGN_BYTE & 0x0F);

    //Need to change back into the IMU mode to use the gyro
    imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.S ensorMode.IMU.bVal & 0x0F);

    sleep(100); //Changing modes again requires a delay
     */
}
