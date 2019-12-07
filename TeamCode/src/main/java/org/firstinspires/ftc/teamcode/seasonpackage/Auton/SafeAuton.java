package org.firstinspires.ftc.teamcode.seasonpackage.Auton;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@Autonomous(name = "Safe Auto")
public class SafeAuton extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        IMU imu = new IMU(hardwareMap, telemetry);

        robot.registerSubsystem(drive);
        robot.registerSubsystem(intake);
        robot.registerSubsystem(imu);

        robot.init();


        waitForStart();
    }

}
