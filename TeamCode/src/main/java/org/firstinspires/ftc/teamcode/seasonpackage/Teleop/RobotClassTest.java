package org.firstinspires.ftc.teamcode.seasonpackage.Teleop;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.ArcadeDrive;

public class RobotClassTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){
        DogeCommander commander = new DogeCommander(this);

        Robot robot = new Robot(this, hardwareMap);

        commander.registerSubsystem(robot);

        commander.init();

        waitForStart();

        commander.runCommand(new ArcadeDrive(robot.drive, gamepad1));
    }
}
