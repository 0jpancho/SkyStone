package org.firstinspires.ftc.teamcode.seasonpackage.Teleop;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.teleopcommands.ArcadeDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

@TeleOp(name = "Doge Teleop Test")
public class DogeTeleopTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode()throws InterruptedException{
        DogeCommander robot = new DogeCommander(this);

        DriveTrain drive = new DriveTrain(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        robot.registerSubsystem(drive);
        robot.registerSubsystem(intake);
        robot.init();

        waitForStart();

        robot.runCommandsParallel(
                new ArcadeDrive(drive, gamepad1)
        );

        robot.stop();
    }
}
