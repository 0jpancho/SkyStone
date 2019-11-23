package org.firstinspires.ftc.teamcode.robot;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.IMU;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.openftc.revextensions2.ExpansionHubEx;


public class Robot implements Subsystem{

    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    private ExpansionHubEx hub;

    public Drive drive;
    public Intake intake;
    public IMU imu;

    public Robot(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initHardware(){
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        imu = new IMU(hardwareMap, opMode.telemetry);
    }

    @Override
    public void periodic() {

        opMode.telemetry.log().setCapacity(12);
        opMode.telemetry.addLine().addData("12 V:", hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
        opMode.telemetry.addLine().addData("12 V:", hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
        opMode.telemetry.addLine().addData("5 V:", hub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
        opMode.telemetry.addLine().addData("Current Draw", hub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));

        opMode.telemetry.update();
    }
}
