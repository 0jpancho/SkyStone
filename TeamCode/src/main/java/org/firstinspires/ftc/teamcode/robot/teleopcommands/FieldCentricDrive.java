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

public class FieldCentricDrive implements Command {

    private Drive drive;
    private Orientation angles;
    private Gamepad driver;

    private double forward, turn, rot;

    public FieldCentricDrive(Drive drive, Gamepad gamepad){
        this.drive = drive;
        this.driver = gamepad;
    }

    @Override
    public void start(){
        drive.setPower(0,0,0,0);
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void periodic(){

        double lTrigger = driver.left_trigger;
        double rTrigger = driver.right_trigger;

        if (lTrigger > .3){
            drive.setPower(-lTrigger, lTrigger, lTrigger, -lTrigger);
        }

        if (rTrigger > .3){
            drive.setPower(rTrigger, -rTrigger, -rTrigger, rTrigger);
        }

        if (driver.right_bumper){
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else{
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (driver.left_bumper){
            drive.zeroHeading();
        }

        forward = -driver.left_stick_y;
        turn = driver.left_stick_x;
        rot = driver.right_stick_x;

        angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (angles.firstAngle < 0){
            angles.firstAngle += 360;
        }

        double heading = Math.toRadians(angles.firstAngle);
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);

        double temp = forward * cos - turn * sin;
        turn = forward * sin - turn * cos;
        forward = temp;


        double rawFLPow = forward - turn + rot;
        double rawBLPow = forward + turn + rot;
        double rawFRPow = forward + turn - rot;
        double rawBRPow = forward - turn -  rot;

        double FLPow = Range.clip(rawFLPow, -1, 1);
        double BLPow = Range.clip(rawBLPow, -1, 1);
        double FRPow = Range.clip(rawFRPow, -1, 1);
        double BRPow = Range.clip(rawBRPow, -1, 1);

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
