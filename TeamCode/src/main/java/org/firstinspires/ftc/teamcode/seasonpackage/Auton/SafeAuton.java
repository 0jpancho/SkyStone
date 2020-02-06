package org.firstinspires.ftc.teamcode.seasonpackage.Auton;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

@Autonomous(name = "Safe Auto")
@Disabled
public class SafeAuton extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        double duration = 1;

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap, telemetry);

        robot.registerSubsystem(drive);

        robot.init();

        double startTime = getRuntime();
        telemetry.addData("Start Time", startTime);

        waitForStart();

        startTime -= getRuntime();
        telemetry.addData("New Start Time", startTime);

        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive() && !isStopRequested() && isStopRequested()){
            while (startTime < duration){
                drive.setPower(0.5, 0.5, 0.5, 0.5);
            }
            drive.setPower(0, 0, 0, 0);
        }
    }
}
