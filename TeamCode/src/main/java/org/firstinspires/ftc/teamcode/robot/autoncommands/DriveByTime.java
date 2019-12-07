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
        LEFT,
        RIGHT
    }

    @Override
    public void start(){
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        switch(direction){

            case FORWARD:

                while (elapsedTime.seconds() < duration){
                    drive.setPower(driveSpeed, driveSpeed, driveSpeed, driveSpeed);
                }

                break;

            case BACKWARD:

                while (elapsedTime.seconds() < duration){
                    drive.setPower(-driveSpeed, -driveSpeed, -driveSpeed, -driveSpeed);
                }

                break;

            case LEFT:

                while (elapsedTime.seconds() < duration){
                    drive.setPower(-driveSpeed, driveSpeed, driveSpeed, -driveSpeed);
                }

                break;
            case RIGHT:

                while (elapsedTime.seconds() < duration){
                    drive.setPower(driveSpeed, -driveSpeed, -driveSpeed, driveSpeed);
                }

                break;
        }
    }

    @Override
    public void periodic(){

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
