package org.firstinspires.ftc.teamcode.seasonpackage.Calibration;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

@TeleOp(name = "Motor Testing")
public class MotorTesting extends LinearOpMode implements DogeOpMode {

    Robot robot;
    Gamepad driver, operator;

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap);
        robot.registerSubsystem(drive);

        robot.init();


        waitForStart();


        while (opModeIsActive() && isStarted() && !isStopRequested()){
            if (driver.a){
                drive.frontLeft.setPower(1);
            }

            if (driver.b){
                drive.backLeft.setPower(1);
            }

            if (driver.y){
                drive.frontRight.setPower(1);
            }

            if (driver.x){
                drive.backRight.setPower(1);
            }

            telemetry.addData("frontLeft Enc", drive.frontLeft.getCurrentPosition());
            telemetry.addData("backLeft Enc", drive.backLeft.getCurrentPosition());
            telemetry.addData("frontRight Enc", drive.frontRight.getCurrentPosition());
            telemetry.addData("backRight Enc", drive.backRight.getCurrentPosition());

            drive.frontLeft.setPower(0);
            drive.backLeft.setPower(0);
            drive.frontRight.setPower(0);
            drive.backRight.setPower(0);

            telemetry.update();
        }

    }

}
