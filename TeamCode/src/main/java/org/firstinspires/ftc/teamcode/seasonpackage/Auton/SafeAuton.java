package org.firstinspires.ftc.teamcode.seasonpackage.Auton;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.autoncommands.DriveByTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@Autonomous(name = "Safe Auto")
public class SafeAuton extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        ElapsedTime elapsedTime = new ElapsedTime();

        double duration = 1;

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap);

        robot.registerSubsystem(drive);

        robot.init();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive() && isStarted() &&  !isStopRequested())
        {
            while (elapsedTime.seconds() < duration){
                drive.setPower(0.75, 0.75, 0.75, 0.75);
            }

            drive.setPower(0, 0, 0, 0);


        }
    }

}
