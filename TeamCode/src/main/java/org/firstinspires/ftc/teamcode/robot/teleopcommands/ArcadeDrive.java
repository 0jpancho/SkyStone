package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveTrain;

import java.util.Arrays;

public class ArcadeDrive implements Command {

    private DriveTrain driveTrain;
    private Gamepad driver;

    private double y = 0;
    private double x = 0;
    private double rot = 0;

    public ArcadeDrive(DriveTrain driveTrain, Gamepad gamepad){
        this.driveTrain = driveTrain;
        this.driver = gamepad;
    }

    @Override
    public void start(){
        driveTrain.setPower(0,0,0,0);
    }

    @Override
    public void periodic(){

        y = driver.left_stick_y;
        x = driver.left_stick_x;
        rot = driver.right_stick_x;

        double frontLeftPower = y - x - rot;
        double backLeftPower =  y + x + rot;
        double frontRightPower = y + x - rot;
        double backRightPower = y - x + rot;

        double[] wheelPowers = {frontLeftPower, backLeftPower, frontRightPower, backRightPower};

        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            frontLeftPower /= wheelPowers[3];
            backLeftPower /= wheelPowers[3];
            frontRightPower /= wheelPowers[3];
            backRightPower /= wheelPowers[3];
        }

        driveTrain.setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    @Override
    public void stop(){
        driveTrain.setPower(0,0,0,0);
    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
