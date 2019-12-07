package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class PowerLift implements Command {

    private Lift lift;
    private Gamepad gamepad;
    private double multiplier;

    public PowerLift(Lift lift, Gamepad gamepad, double multiplier){
        this.lift = lift;
        this.gamepad = gamepad;
        this.multiplier = multiplier;
    }

    @Override
    public void start(){
        lift.setLiftPower(0);
    }

    @Override
    public void periodic(){
        lift.setLiftPower(gamepad.right_stick_y * multiplier);
    }

    @Override
    public void stop(){
        lift.setLiftPower(0);
    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
