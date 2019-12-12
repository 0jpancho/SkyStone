package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.Gripper;

public class RunGripper implements Command {

    private Gripper gripper;
    private Gamepad operator;

    private boolean togglePivot = false;
    private boolean toggleGripper;

    public RunGripper(Gripper gripper, Gamepad gamepad){
        this.gripper = gripper;
        this.operator = gamepad;
    }

    @Override
    public void start(){
        gripper.setPivotState(Gripper.PivotState.STOW);
        gripper.setGripperState(Gripper.GripperState.GRIP);
    }

    @Override
    public void periodic(){

        if (operator.x){
            gripper.setPivotState(Gripper.PivotState.STOW);
        }

        else if (operator.b){
            gripper.setPivotState(Gripper.PivotState.DEPLOY);
        }

        if(operator.a){
            gripper.setGripperState(Gripper.GripperState.GRIP);
        }

        else if (operator.y){
            gripper.setGripperState(Gripper.GripperState.RELEASE);

        }
    }

    @Override
    public void stop(){

    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
