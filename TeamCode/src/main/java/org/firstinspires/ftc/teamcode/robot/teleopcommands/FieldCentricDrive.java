package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.util.Constants;

public class FieldCentricDrive implements Command {

    private Drive drive;
    private IMU imu;
    private Orientation angles;
    private Gamepad driver;

    private double forward, turn, rot;

    public FieldCentricDrive(Drive drive, IMU imu, Gamepad gamepad){
        this.drive = drive;
        this.imu = imu;
        this.driver = gamepad;
    }

    @Override
    public void start(){
        drive.setPower(0,0,0,0);
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void periodic(){

        forward = -driver.left_stick_y;
        turn = driver.left_stick_x;
        rot = driver.right_stick_x;

        double rawFLPow;
        double rawBLPow;
        double rawFRPow;
        double rawBRPow;

        double FLPow;
        double BLPow;
        double FRPow;
        double BRPow;

        angles = imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle < 0){
            angles.firstAngle += 360;
        }

        double heading = Math.toRadians(angles.firstAngle);
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);

        double temp = forward * cos - turn * sin;
        turn = forward * sin - turn * cos;
        forward = temp;


        rawFLPow = forward + turn + rot;
        rawBLPow = forward + turn - rot;
        rawFRPow = forward - turn - rot;
        rawBRPow = forward - turn + rot;

        FLPow = Range.clip(rawFLPow, -1, 1);
        BLPow = Range.clip(rawBLPow, -1, 1);
        FRPow = Range.clip(rawFRPow, -1, 1);
        BRPow = Range.clip(rawBRPow, -1, 1);

        drive.setPower(FLPow, BLPow, FRPow, BRPow);
    }

    @Override
    public void stop(){
        drive.setPower(0,0,0,0);
    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
