package org.firstinspires.ftc.teamcode.seasonpackage.Teleop;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.ArcadeDrive;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.RunIntake;

@TeleOp(name = "Doge Teleop Test", group = "Teleop")
public class DogeTeleopTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode()throws InterruptedException{
        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        IMU imu = new IMU(hardwareMap, telemetry);

        robot.registerSubsystem(drive);
        robot.registerSubsystem(intake);
        robot.registerSubsystem(imu);

        robot.init();

        waitForStart();

        robot.runCommandsParallel(
                //new FieldCentricDrive(drive, imu, gamepad1),
                new ArcadeDrive(drive, gamepad1),
                new RunIntake(intake, gamepad2)
        );

        robot.stop();
    }
}
