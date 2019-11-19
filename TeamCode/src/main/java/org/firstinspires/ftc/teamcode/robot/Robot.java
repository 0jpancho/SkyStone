package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardware.IMU;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
//

public class Robot {

    public DriveTrain drive;
    public Intake intake;

    public IMU imu;

    public Telemetry t;

    //public Lift lift;

    public Robot(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry) {

        t = telemetry;

        drive = new DriveTrain(opMode, hardwareMap, telemetry);
        intake = new Intake(opMode, hardwareMap, telemetry);

        imu = new IMU(hardwareMap, telemetry);

        robotTelemetry();
        //lift = new Lift(opMode, hardwareMap, telemetry);
    }

    public void robotTelemetry(){
        imu.imuTelemetry();

        t.update();
    }
}
