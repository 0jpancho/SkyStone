package org.firstinspires.ftc.teamcode.robot.autoncommands;

import com.disnodeteam.dogecommander.Command;
import com.disnodeteam.dogecommander.Subsystem;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;

public class AcquireSkyStone implements Command {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private SkystoneDetector detector;
    private OpenCvCamera phoneCam;

    public AcquireSkyStone(HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        detector = new SkystoneDetector();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        detector = new SkystoneDetector();
        phoneCam.setPipeline(detector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
    }

    @Override
    public void start() {
        telemetry.addData("Stone Position X", detector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", detector.getScreenPosition().y);
        telemetry.addData("Frame Count", phoneCam.getFrameCount());
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
        telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
        telemetry.update();

        /*
         * NOTE: stopping the stream from the camera early (before the end of the OpMode
         * when it will be automatically stopped for you) *IS* supported. The "if" statement
         * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
         */
    }

    @Override
    public void periodic(){

    }

    @Override
    public void stop(){
        phoneCam.stopStreaming();
    }

    @Override
    public boolean isCompleted(){
        return false;
    }
}
