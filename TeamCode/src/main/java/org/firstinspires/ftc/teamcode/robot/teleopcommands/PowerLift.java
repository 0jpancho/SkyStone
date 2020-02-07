package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class PowerLift implements Command {

    private Lift lift;
    private Gamepad operator;
    private double upMultiplier, downMultiplier;

    public PowerLift(Lift lift, Gamepad operator, double upMultiplier, double downMultiplier){
        this.lift = lift;
        this.operator = operator;
        this.upMultiplier = upMultiplier;
        this.downMultiplier = downMultiplier;
    }

    @Override
    public void start(){

    }

    @Override
    public void periodic(){

        if (operator.left_stick_y < 0){
            lift.setLiftPower(operator.left_stick_y * upMultiplier);
        }

        if (operator.left_stick_y > 0){
            lift.setLiftPower(operator.left_stick_y * downMultiplier);
        }
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
