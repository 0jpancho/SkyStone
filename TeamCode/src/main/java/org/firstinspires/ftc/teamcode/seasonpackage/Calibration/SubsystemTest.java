package org.firstinspires.ftc.teamcode.seasonpackage.Calibration;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.PowerLift;
import org.firstinspires.ftc.teamcode.robot.teleopcommands.RunManipulator;

@TeleOp(name = "Subsystems Test", group = "Subsystems Test")
@Disabled
public class SubsystemTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Lift lift = new Lift(hardwareMap);
        Manipulator manipulator = new Manipulator(hardwareMap);

        robot.registerSubsystem(lift);
        robot.registerSubsystem(manipulator);

        robot.init();

        waitForStart();

        robot.runCommandsParallel(
                new RunManipulator(manipulator, gamepad2),
                new PowerLift(lift, gamepad2, 0.5)
        );
    }
}
