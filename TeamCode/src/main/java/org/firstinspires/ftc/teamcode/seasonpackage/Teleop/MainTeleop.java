package org.firstinspires.ftc.teamcode.seasonpackage.Teleop;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.ArcadeDrive;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.PowerLift;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.RunIntake;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.RunManipulator;

@TeleOp(name = "Main Teleop", group = "Teleop")
public class MainTeleop extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Manipulator gripper = new Manipulator(hardwareMap);

        robot.registerSubsystem(drive);
        robot.registerSubsystem(intake);
        robot.registerSubsystem(lift);
        robot.registerSubsystem(gripper);

        robot.init();

        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        drive.setGrabberState(Drive.GrabberState.STOW);

        robot.runCommandsParallel(
                    //new FieldCentricDrive(drive, gamepad1),
                    new ArcadeDrive(drive, gamepad1),
                    new RunIntake(intake, gamepad2),
                    new PowerLift(lift, gamepad2, 1, 0.25),
                    new RunManipulator(gripper, gamepad2)
            );

        if(isStopRequested()){
            robot.stop();
            stop();
        }
    }
}
