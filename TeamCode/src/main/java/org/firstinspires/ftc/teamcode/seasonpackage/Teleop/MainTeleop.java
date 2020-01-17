package org.firstinspires.ftc.teamcode.seasonpackage.Teleop;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.HardwareKeys;
import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.ArcadeDrive;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.PowerLift;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.RunIntake;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.RunManipulator;

@TeleOp(name = "Teleop", group = "Teleop")
public class MainTeleop extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Manipulator gripper = new Manipulator(hardwareMap);
        IMU imu = new IMU(hardwareMap, telemetry);

        robot.registerSubsystem(drive);
        robot.registerSubsystem(intake);
        robot.registerSubsystem(lift);
        robot.registerSubsystem(gripper);
        robot.registerSubsystem(imu);

        robot.init();

        waitForStart();

        robot.runCommandsParallel(
                //new FieldCentricDrive(drive, imu, gamepad1),
                new ArcadeDrive(drive, gamepad1),
                new RunIntake(intake, gamepad2),
                new PowerLift(lift, gamepad2, 0.5),
                new RunManipulator(gripper, gamepad2)
        );

        robot.stop();
    }
}
