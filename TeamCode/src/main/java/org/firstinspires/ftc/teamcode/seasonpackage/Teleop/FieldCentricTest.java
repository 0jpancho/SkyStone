package org.firstinspires.ftc.teamcode.seasonpackage.Teleop;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.FieldCentricDrive;

@TeleOp(name = "Field Centric Test")
public class FieldCentricTest extends LinearOpMode implements DogeOpMode{

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap, telemetry);

        robot.registerSubsystem(drive);

        robot.init();

        waitForStart();

        if (isStopRequested() || !opModeIsActive()){
            robot.stop();
        }
        else{
            robot.runCommandsParallel(
                    new FieldCentricDrive(drive, gamepad1)
            );
        }
    }
}
