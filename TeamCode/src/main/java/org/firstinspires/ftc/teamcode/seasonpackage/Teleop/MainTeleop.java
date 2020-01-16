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
import org.firstinspires.ftc.teamcode.robot.teleopcommands.ArcadeDrive;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.RunIntake;

@TeleOp(name = "Teleop", group = "Teleop")
public class MainTeleop extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){
        DcMotor lift;
        lift = hardwareMap.dcMotor.get(HardwareKeys.LIFT_NAME);

        Servo pivotL, pivotR, gripperArm;

        pivotL = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_L);
        pivotR = hardwareMap.servo.get(HardwareKeys.GRIPPER_PIVOT_R);

        pivotR.setDirection(Servo.Direction.REVERSE);

        gripperArm = hardwareMap.servo.get(HardwareKeys.GRIPPER_ARM);

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        //Lift lift = new Lift(hardwareMap);
        //Manipulator gripper = new Manipulator(hardwareMap);
        IMU imu = new IMU(hardwareMap, telemetry);

        robot.registerSubsystem(drive);
        robot.registerSubsystem(intake);
        //robot.registerSubsystem(lift);
        //robot.registerSubsystem(gripper);
        robot.registerSubsystem(imu);

        robot.init();

        waitForStart();

        robot.runCommandsParallel(
                //new FieldCentricDrive(drive, imu, gamepad1),
                new ArcadeDrive(drive, gamepad1),

                new RunIntake(intake, gamepad2)
                //new PowerLift(lift, gamepad2, 0.5)
                //new RunManipulator(gripper, gamepad2)
        );

        if (gamepad2.x)
        {
            pivotL.setPosition(0);
            pivotR.setPosition(0);
        }

        else if (gamepad2.b){
            pivotL.setPosition(1);
            pivotR.setPosition(1);
        }

        if(gamepad2.dpad_up){
            gripperArm.setPosition(0);
        }

        else if (gamepad2.dpad_down){
            gripperArm.setPosition(1);
        }
        robot.stop();
    }
}
