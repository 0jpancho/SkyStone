package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class PowerLift implements Command {

    private Lift lift;
    private Gamepad operator;
    private double multiplier;

    public PowerLift(Lift lift, Gamepad operator, double multiplier){
        this.lift = lift;
        this.operator = operator;
        this.multiplier = multiplier;
    }

    @Override
    public void start(){

    }

    @Override
    public void periodic(){
        lift.setLiftPower(operator.left_stick_y * multiplier);
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
