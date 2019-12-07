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
            if (togglePivot){
                gripper.setPivotState(Gripper.PivotState.STOW);
                togglePivot = false;
            }
        }

        else if (!togglePivot){
            gripper.setPivotState(Gripper.PivotState.DEPLOY);
            togglePivot = true;
        }

        if(operator.b){
            if(toggleGripper){
                gripper.setGripperState(Gripper.GripperState.GRIP);
                toggleGripper = false;
            }
        }

        else if (!toggleGripper){
            gripper.setGripperState(Gripper.GripperState.RELEASE);
            toggleGripper = true;
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
