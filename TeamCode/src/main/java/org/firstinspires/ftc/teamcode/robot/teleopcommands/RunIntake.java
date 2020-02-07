package org.firstinspires.ftc.teamcode.robot.teleopcommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public class RunIntake implements Command {

    private Intake intake;
    private Gamepad operator;

    public RunIntake(Intake intake, Gamepad gamepad){
        this.intake = intake;
        this.operator = gamepad;
    }

    @Override
    public void start(){
        intake.setPowerState(Intake.PowerState.STOP);
        intake.setPivotState(Intake.PivotState.STOW);
    }

    @Override
    public void periodic(){
        if (operator.left_bumper){
            intake.setPowerState(Intake.PowerState.INTAKE);
        }
        else if (operator.right_bumper){
            intake.setPowerState(Intake.PowerState.SPIT_OUT);
        }
        else{
            intake .setPowerState(Intake.PowerState.STOP);
        }

        if (operator.dpad_up){
            intake.setPivotState(Intake.PivotState.DEPLOY);
        }
        else if (operator.dpad_down ){
            intake.setPivotState(Intake.PivotState.STOW);
        }
    }

    @Override
    public void stop(){
        intake.setPowerState(Intake.PowerState.STOP);
        intake.setPivotState(Intake.PivotState.STOW);
    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
