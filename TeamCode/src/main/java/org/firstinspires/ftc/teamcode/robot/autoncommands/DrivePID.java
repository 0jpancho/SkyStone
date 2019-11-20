package org.firstinspires.ftc.teamcode.robot.autoncommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.SynchronousPID;

public class DrivePID implements Command {

    private Drive drive;
    private Telemetry t;

    double maxSpeed = 0;
    double p = 0;
    double i = 0;
    double d = 0;
    double inches = 0;
    long delay = 0;
    double left = 0;
    double right = 0;

    PIDFCoefficients coefficients;

    double targetPulses;

    int leftEncoder = 0;
    int rightEncoder = 0;

    SynchronousPID leftPID;
    SynchronousPID rightPID;

    public DrivePID (Drive drive, double maxSpeed, double p, double i, double d, double inches, long delay, PIDFCoefficients coefficients, Telemetry telemetry){
        this.drive = drive;
        this.t = telemetry;

        this.maxSpeed = maxSpeed;

        this.p = p;
        this.i = i;
        this.d = d;

        this.inches = inches;
        this.delay = delay;
        this.coefficients = coefficients;

        leftPID = new SynchronousPID(p, i, d);
        rightPID = new SynchronousPID(p, i, d);

        targetPulses = inches * Constants.countsPerInch();

        drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPIDFCoeffs(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    @Override
    public void start(){
        drive.setPower(0,0,0,0);

        leftEncoder = drive.leftStraightPair.getEncoder();
        rightEncoder = drive.rightStraightPair.getEncoder();

        leftPID.setOutputRange(-maxSpeed, maxSpeed);
        leftPID.setSetpoint(targetPulses);

        rightPID.setOutputRange(-maxSpeed, maxSpeed);
        rightPID.setSetpoint(targetPulses);

        left = leftPID.calculate(leftEncoder);
        right = rightPID.calculate(rightEncoder);

        t.addData("Left PID State", leftPID.getState());
        t.addData("Left Setpoint", leftPID.getSetpoint());

        t.addData("Right PID State", rightPID.getState());
        t.addData("Right Setpoint", rightPID.getSetpoint());

        t.addData("Front Left Enc", drive.frontLeft.getCurrentPosition());
        t.addData("Back Left Enc", drive.backLeft.getCurrentPosition());
        t.addData("Front Right Enc", drive.frontRight.getCurrentPosition());
        t.addData("Back Right Enc", drive.backRight.getCurrentPosition());

        drive.setStrDrive(left, right);

        t.update();
    }

    @Override
    public void periodic(){

    }

    @Override
    public void stop(){
        drive.stopStrMotors();
    }

    @Override
    public boolean isCompleted(){
        return (Math.abs(leftEncoder - targetPulses) < 25) || !(Math.abs(rightEncoder - targetPulses) < 25);
    }
}
