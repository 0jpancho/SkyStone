package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

import java.util.Arrays;

public class ArcadeDrive implements Command {

    private Drive drive;
    private Gamepad driver;

    private double y = 0;
    private double x = 0;
    private double rot = 0;

    private boolean toggleSpeed;

    public ArcadeDrive(Drive drive, Gamepad gamepad){
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

        y = driver.left_stick_y;
        x = driver.left_stick_x;
        rot = driver.right_stick_x;

        double frontLeftPower = y - x - rot;
        double backLeftPower =  y + x - rot;
        double frontRightPower = y + x + rot;
        double backRightPower = y - x + rot;

        if (driver.right_bumper){
            if (toggleSpeed){

                frontLeftPower = Range.clip(frontLeftPower, -1, 1);
                backLeftPower = Range.clip(backLeftPower, -1, 1);
                frontRightPower = Range.clip(frontRightPower, -1, 1);
                backRightPower = Range.clip(backRightPower, -1, 1);

                toggleSpeed = false;
            }
        }

        else if (!toggleSpeed){
            frontLeftPower = Range.clip(frontLeftPower, -.5, 0.5);
            backLeftPower = Range.clip(backLeftPower, -.5, 0.5);
            frontRightPower = Range.clip(frontRightPower, -.5, 0.5);
            backRightPower = Range.clip(backRightPower, -.5, 0.5);

            toggleSpeed = true;
        }

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
