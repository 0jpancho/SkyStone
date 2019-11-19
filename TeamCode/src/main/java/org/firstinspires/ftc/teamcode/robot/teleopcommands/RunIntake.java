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
        intake.setState(Intake.State.STOP);
        intake.setPivotPow(0);
    }

    @Override
    public void periodic(){
        if (operator.left_bumper){
            intake.setState(Intake.State.INTAKE);
        }
        else if (operator.right_bumper){
            intake.setState(Intake.State.SPIT_OUT);
        }

        intake.setPivotPow(operator.left_stick_y);
    }

    @Override
    public void stop(){
        intake.setState(Intake.State.STOP);
        intake.setPivotPow(0);
    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
