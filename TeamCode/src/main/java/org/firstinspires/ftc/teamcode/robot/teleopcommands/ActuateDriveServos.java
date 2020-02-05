package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

public class ActuateDriveServos implements Command {

    Drive drive;
    Gamepad driver;

    public ActuateDriveServos(Drive drive){

    }

    @Override
    public void start(){

    }

    @Override
    public void periodic(){

    }

    @Override
    public void stop(){

    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
