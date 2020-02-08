package org.firstinspires.ftc.teamcode.seasonpackage.Auton;

import com.disnodeteam.dogecommander.DogeCommander;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.autoncommands.DriveByTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;

@Autonomous(name = "Stone Color Test")
public class ColorStoneTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){
        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap, telemetry);
        robot.registerSubsystem(drive);
        robot.init();

        ElapsedTime elapsedTime = new ElapsedTime();

        //Start routine - strafe to stones
        robot.runCommand(
            new DriveByTime(drive, elapsedTime, 0.5, 3, DriveByTime.Direction.STRAFE_LEFT)
        );

        //Move to first stone nearest to mid field
        robot.runCommand(
            new DriveByTime(drive, elapsedTime, 0.25, 1, DriveByTime.Direction.FORWARD)
        );


        if(isStopRequested()){
            robot.stop();
            stop();
        }
    }
}
