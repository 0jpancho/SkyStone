package org.firstinspires.ftc.teamcode.seasonpackage.Auton;

import com.disnodeteam.dogecommander.DogeCommander;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.disnodeteam.dogecommander.DogeOpMode;

import org.firstinspires.ftc.teamcode.robot.autoncommands.DrivePID;
import org.firstinspires.ftc.teamcode.robot.autoncommands.TurnPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Constants;

@Autonomous(name = "Doge Auton Test", group = "Autonomous")
public class PIDAutonTest extends LinearOpMode implements DogeOpMode {

    @Override
    public void runOpMode(){

        DogeCommander robot = new DogeCommander(this);

        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        IMU imu = new IMU(hardwareMap, telemetry);

        robot.registerSubsystem(drive);
        robot.registerSubsystem(intake);
        robot.registerSubsystem(imu);

        robot.init();

        waitForStart();

        robot.runCommand(new DrivePID(drive, 0.5, 0.1, 0, 0, 12, 0, Constants.autoDrive, telemetry));

        robot.runCommand(new TurnPID(drive, 0.5, 0.1, 0,0, 90, imu, 5, Constants.autoTurn, telemetry));
    }
}
