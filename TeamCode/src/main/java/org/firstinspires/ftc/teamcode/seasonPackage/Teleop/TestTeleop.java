package org.firstinspires.ftc.teamcode.seasonPackage.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name = "Teleop")
public class TestTeleop extends LinearOpMode {

    Robot robot;
    Gamepad driver, operator;

    private static double DRIVE_SPEED = Constants.NORMAL_SPEED;

    @Override
    public void runOpMode(){

        driver = gamepad1;
        operator = gamepad2;

        robot = new Robot(this, hardwareMap, telemetry);
        //Thread strafeThread = new StrafeThread();

        telemetry.update();

        waitForStart();

        //strafeThread.start();

        while (opModeIsActive() && !isStopRequested() && isStarted()) {

            robot.drive.fieldCentricDrive(-driver.left_stick_y, driver.left_stick_x,
                                            driver.right_stick_x, robot.imu);

            robot.intake.intakeOuttake(operator.left_bumper, operator.right_bumper);
            robot.intake.pivotIntake(operator.left_stick_y);

            //robot.lift.moveLift(operator.right_stick_y);

            robot.robotTelemetry();
        }
    }
}
