package org.firstinspires.ftc.teamcode.robot.autoncommands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.util.SynchronousPID;

public class TurnPID implements Command {

    private Drive drive;
    private IMU imu;
    private Telemetry t;

    private double maxTurn = 0;

    private double p = 0;
    private double i = 0;
    private double d = 0;

    private double angle = 0;
    private double turnTolerance = 0;

    private double heading = 0;
    private double turnFactor = 0;

    private PIDFCoefficients coefficients;

    private SynchronousPID turnPID;

    public TurnPID(Drive drive, double maxTurn, double p, double i, double d, double angle, IMU imu,
                   double turnTolerance, PIDFCoefficients coefficients, Telemetry telemetry){

        this.drive = drive;
        this.imu = imu;
        this.t = telemetry;

        this.maxTurn = maxTurn;
        this.angle = angle;
        this.turnTolerance = turnTolerance;

        this.p = p;
        this.i = i;
        this.d = d;

        turnPID = new SynchronousPID(this.p, this.i, this.d);

        this.coefficients = coefficients;

        imu.zeroHeading();

        drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //drive.setPIDFCoeffs(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    @Override
    public void start(){
        drive.setPower(0,0,0,0);

        turnPID.setOutputRange(-maxTurn, maxTurn);
        turnPID.setSetpoint(angle);
        turnPID.setDeadband(turnTolerance);

        heading = imu.getHeading();
        turnFactor = turnPID.calculateGivenError(angle - imu.getHeading());

        /*
        if (turnFactor > -.07 && turnFactor < 0) {
            turnFactor = -.07;
        }
        if (turnFactor < .07 && turnFactor > 0) {
            turnFactor = .07;
        }
        */


        drive.setStrDrive(turnFactor, -turnFactor);

        t.addData("Heading", heading);
        t.addData("Turn error", turnPID.getError());
        t.addData("TurnFactor", turnFactor);
        t.update();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void stop(){
        drive.setPower(0,0,0,0);
    }

    @Override
    public boolean isCompleted(){
        return (turnPID.onTarget(2));
    }
}
