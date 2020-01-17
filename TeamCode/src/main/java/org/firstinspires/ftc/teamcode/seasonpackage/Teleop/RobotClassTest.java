package org.firstinspires.ftc.teamcode.seasonpackage.Teleop;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.ArcadeDrive;

@TeleOp(name = "Robot Class Test")
@Disabled
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
