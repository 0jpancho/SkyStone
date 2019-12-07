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

    private double maxSpeed = 0;
    private double p = 0;
    private double i = 0;
    private double d = 0;
    private double inches = 0;
    private long delay = 0;

    private double left = 0;
    private double right = 0;

    private PIDFCoefficients coefficients;

    private double targetPulses;

    private int leftEncoder = 0;
    private int rightEncoder = 0;

    private SynchronousPID leftPID;
    private SynchronousPID rightPID;

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

        //drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //drive.setPIDFCoeffs(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    @Override
    public void start(){
        drive.setPower(0,0,0,0);
        drive.leftStrafePair.resetEncoder();
        drive.rightStrafePair.resetEncoder();

        leftEncoder = drive.leftStraightPair.getEncoder();
        rightEncoder = drive.rightStraightPair.getEncoder();

        leftPID.setOutputRange(-maxSpeed, maxSpeed);
        leftPID.setSetpoint(targetPulses);

        rightPID.setOutputRange(-maxSpeed, maxSpeed);
        rightPID.setSetpoint(targetPulses);

        left = leftPID.calculate(leftEncoder);
        right = rightPID.calculate(rightEncoder);

        t.addData("Left PID PowerState", leftPID.getState());
        t.addData("Left Setpoint", leftPID.getSetpoint());

        t.addData("Right PID PowerState", rightPID.getState());
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
