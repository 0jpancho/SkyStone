package org.firstinspires.ftc.teamcode.seasonpackage.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode(){

        Robot robot = new Robot(TestAuto.this, hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            robot.drive.driveForwardsBackwards(0.5,0.1,0,0,12, 0);
            robot.drive.turnPID(0.25, 0.1, 0, 0, 45, robot.imu, 50, 2);
        }
    }
}
