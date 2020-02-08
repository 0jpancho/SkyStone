package org.firstinspires.ftc.teamcode.robot.autoncommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

public class DriveByTime implements Command {

    //Declare hardware
    private Drive drive;
    private ElapsedTime elapsedTime;
    private double driveSpeed;
    private double duration;

    private Direction direction;

    public DriveByTime(Drive drive, ElapsedTime elapsedTime, double driveSpeed, double duration, Direction direction){
        this.drive = drive;
        this.elapsedTime = elapsedTime;
        this.driveSpeed = driveSpeed;
        this.duration = duration;
        this.direction = direction;
    }

    //Direction states
    public enum Direction {
        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT
    }

    @Override
    public void start(){
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {

        //Set motor powers on command loop
        switch (direction) {

            case FORWARD:
                drive.setPower(driveSpeed, driveSpeed, driveSpeed, driveSpeed);
                break;

            case BACKWARD:
                drive.setPower(-driveSpeed, -driveSpeed, -driveSpeed, -driveSpeed);
                break;

            case STRAFE_LEFT:
                drive.setPower(-driveSpeed, driveSpeed, driveSpeed, -driveSpeed);
                break;

            case STRAFE_RIGHT:
                drive.setPower(driveSpeed, -driveSpeed, -driveSpeed, driveSpeed);
                break;
        }
    }

    @Override
    public void stop(){
        drive.setPower(0, 0, 0, 0);
    }

    @Override
    public boolean isCompleted(){
        return elapsedTime.seconds() >= duration;
    }
}
