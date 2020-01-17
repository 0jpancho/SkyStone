package org.firstinspires.ftc.teamcode.robot.autoncommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

public class DriveByTime implements Command {

    private Drive drive;
    private ElapsedTime elapsedTime;
    private double driveSpeed;
    private double duration;

    private DirectionState direction;

    public DriveByTime(Drive drive, ElapsedTime elapsedTime, double driveSpeed, double duration, DirectionState direction){
        this.drive = drive;
        this.elapsedTime = elapsedTime;
        this.driveSpeed = driveSpeed;
        this.duration = duration;
        this.direction = direction;
    }

    public enum DirectionState{
        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT
    }

    @Override
    public void start(){
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    @Override
    public void periodic() {
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
        elapsedTime.reset();
        drive.setPower(0, 0, 0, 0);
    }

    @Override
    public boolean isCompleted(){
        return elapsedTime.seconds() >= duration;
    }
}
