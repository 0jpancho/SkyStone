package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.Manipulator;

public class RunManipulator implements Command {

    private Manipulator manipulator;
    private Gamepad operator;

    boolean pivotToggle = false;
    boolean inverterToggle = false;
    boolean gripperToggle = false;

    public RunManipulator(Manipulator manipulator, Gamepad gamepad){
        this.manipulator = manipulator;
        this.operator = gamepad;
    }

    @Override
    public void start(){
        manipulator.setPivotState(Manipulator.PivotState.STOW);
        manipulator.setInverterState(Manipulator.InverterState.STOW);
        manipulator.setGripperState(Manipulator.GripperState.GRIP);
    }

    @Override
    public void periodic(){

        if(operator.x && !pivotToggle) {
            if(manipulator.pivotState.equals(Manipulator.PivotState.STOW) ) {
                manipulator.setPivotState(Manipulator.PivotState.DEPLOY);
            }
            else {
                manipulator.setPivotState(Manipulator.PivotState.STOW);
            }
            pivotToggle = true;
        }
        else if(!operator.a){
            pivotToggle = false;
        }

        if(operator.a && !inverterToggle){
            if(manipulator.inverterState.equals(Manipulator.InverterState.STOW)){
                manipulator.setInverterState(Manipulator.InverterState.FLIP);
            }
            else{
                manipulator.setInverterState(Manipulator.InverterState.STOW);
            }
            inverterToggle = true;
        }
        else if(!operator.a){
            inverterToggle = false;
        }

        if(operator.b && !gripperToggle){
            if(manipulator.gripperState.equals(Manipulator.GripperState.RELEASE)){
                manipulator.setGripperState(Manipulator.GripperState.GRIP);
            }
            else{
                manipulator.setGripperState(Manipulator.GripperState.RELEASE);
            }
            gripperToggle = true;
        }
        else if (operator.b){
            gripperToggle = false;
        }

         //manipulator.setPivotPower(-operator.right_stick_y, 0.5);
    }

    @Override
    public void stop(){
    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
