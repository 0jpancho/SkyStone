package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.util.Constants;

public class FieldCentricDrive implements Command {

    private Drive drive;
    private IMU imu;
    private Gamepad driver;

    private double y, x, rot;

    public FieldCentricDrive(Drive drive, IMU imu, Gamepad gamepad){
        this.drive = drive;
        this.imu = imu;
        this.driver = gamepad;
    }

    @Override
    public void start(){
        drive.setPower(0,0,0,0);
    }

    @Override
    public void periodic(){

        y = -driver.left_stick_y;
        x = driver.left_stick_x;
        rot = driver.right_stick_x;

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
