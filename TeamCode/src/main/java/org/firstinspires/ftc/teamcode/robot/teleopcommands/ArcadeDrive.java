package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

public class ArcadeDrive implements Command {

    private Drive drive;
    private Gamepad driver;

    private double forward = 0;
    private double strafe = 0;
    private double rot = 0;

    public ArcadeDrive(Drive drive, Gamepad gamepad){
        this.drive = drive;
        this.driver = gamepad;
    }

    @Override
    public void start(){
        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        forward = driver.left_stick_y;
        strafe = driver.left_stick_x;
        rot = driver.right_stick_x;

        double frontLeftPower = forward - strafe + rot;
        double backLeftPower =  forward + strafe + rot;
        double frontRightPower = forward + strafe - rot;
        double backRightPower = forward - strafe - rot;

        frontLeftPower = Range.clip(frontLeftPower, -1, 1);
        backLeftPower = Range.clip(backLeftPower, -1, 1);
        frontRightPower = Range.clip(frontRightPower, -1, 1);
        backRightPower = Range.clip(backRightPower, -1, 1);

        drive.setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
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
