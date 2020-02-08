package org.firstinspires.ftc.teamcode.seasonpackage.Auton;

import com.disnodeteam.dogecommander.DogeCommander;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.disnodeteam.dogecommander.DogeOpMode;

import org.firstinspires.ftc.teamcode.robot.autoncommands.TurnPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Constants;

@Autonomous(name = "Doge Auton Test", group = "Autonomous")
@Disabled
public class PIDTurnTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap);

        robot.registerSubsystem(drive);
        robot.registerSubsystem(intake);

        robot.init();

        waitForStart();

        robot.runCommand(new TurnPID(drive, 0.5, 0.1, 0,0, 90, 5, Constants.autoTurn, telemetry));
    }
}
