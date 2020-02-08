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
        if (operator.left_trigger > 0.1){
            lift.setLiftPower(-operator.left_trigger * upMultiplier);
        }

        else if (operator.right_trigger > 0.1){
            lift.setLiftPower(operator.left_trigger * downMultiplier);
        }

        else{
            lift.setLiftPower(0);
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
