package org.firstinspires.ftc.teamcode.seasonpackage.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Motor Testing")
public class MotorTesting extends LinearOpMode {

    Robot robot;
    Gamepad driver, operator;

    @Override
    public void runOpMode(){

        driver = gamepad1;
        operator = gamepad2;

        robot = new Robot(this,hardwareMap,telemetry);

        telemetry.update();

        waitForStart();

        /*
        while (opModeIsActive() && isStarted() && !isStopRequested()){
            if (driver.a){
                robot.drive.frontLeft.setPower(1);
            }

            if (driver.b){
                robot.drive.backLeft.setPower(1);
            }

            if (driver.y){
                robot.drive.frontRight.setPower(1);
            }

            if (driver.x){
                robot.drive.backRight.setPower(1);
            }

            telemetry.addData("frontLeft Enc", robot.drive.frontLeft.getCurrentPosition());
            telemetry.addData("backLeft Enc", robot.drive.backLeft.getCurrentPosition());
            telemetry.addData("frontRight Enc", robot.drive.frontRight.getCurrentPosition());
            telemetry.addData("backRight Enc", robot.drive.backRight.getCurrentPosition());

            robot.drive.frontLeft.setPower(0);
            robot.drive.backLeft.setPower(0);
            robot.drive.frontRight.setPower(0);
            robot.drive.backRight.setPower(0);

            telemetry.update();
        }
        */
    }

}
