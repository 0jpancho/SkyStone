package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Intake {

    DcMotor left, right, pivot;
    LinearOpMode l;
    Telemetry t;

    public Intake(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry){

        l = opMode;
        t = telemetry;

        left = hardwareMap.dcMotor.get(HardwareKeys.LEFT_NAME);
        right = hardwareMap.dcMotor.get(HardwareKeys.RIGHT_NAME);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot = hardwareMap.dcMotor.get(HardwareKeys.PIVOT_NAME);
    }

    public void intakeOuttake(boolean forward, boolean backward){
        if(forward && l.opModeIsActive()){
            left.setPower(Constants.INTAKE_SPEED);
            right.setPower(Constants.INTAKE_SPEED);
        }

        else if (backward && l.opModeIsActive()){
            left.setPower(Constants.OUTTAKE_SPEED);
            right.setPower(Constants.OUTTAKE_SPEED);
        }

        else {
            left.setPower(0);
            right.setPower(0);
        }
    }

    public void pivotIntake(double inputPower){
        pivot.setPower(inputPower * Constants.PIVOT_MULTIPLIER);
    }
}
