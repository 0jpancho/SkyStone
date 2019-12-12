package org.firstinspires.ftc.teamcode.seasonpackage.Calibration;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.PowerLift;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.RunGripper;

@TeleOp(name = "Subsystems Test", group = "Subsystems Test")
public class SubsystemTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Lift lift = new Lift(hardwareMap);
        Gripper gripper = new Gripper(hardwareMap);

        robot.registerSubsystem(lift);
        robot.registerSubsystem(gripper);

        robot.init();

        waitForStart();

        robot.runCommandsParallel(
                new RunGripper(gripper, gamepad2),
                new PowerLift(lift, gamepad2, 0.5)
        );
    }
}
