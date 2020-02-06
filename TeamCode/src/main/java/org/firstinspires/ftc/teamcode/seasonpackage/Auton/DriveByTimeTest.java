package org.firstinspires.ftc.teamcode.seasonpackage.Auton;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.autoncommands.DriveByTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

@Autonomous(name = "DriveByTime Test")
public class DriveByTimeTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        ElapsedTime elapsedTime = new ElapsedTime();

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap, telemetry);

        robot.registerSubsystem(drive);

        robot.init();

        waitForStart();

        robot.runCommand(
                new DriveByTime(drive, elapsedTime, 0.5, 1, DriveByTime.Direction.FORWARD)
        );

        robot.stop();
    }
}
