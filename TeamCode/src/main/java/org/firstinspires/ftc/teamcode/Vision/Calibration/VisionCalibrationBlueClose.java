package org.firstinspires.ftc.teamcode.Vision.Calibration;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.PropDetectionBlueClose;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Config
@Autonomous(name="Vision Calibration Blue Close")
public class VisionCalibrationBlueClose extends LinearOpMode {
    private VisionPortal portal;
    private PropDetectionBlueClose processor;

    @Override
    public void runOpMode() throws InterruptedException {
        processor = new PropDetectionBlueClose();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .build();

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("Detection", processor.detection);
            telemetry.addData("Left value", processor.leftSum);
            telemetry.addData("Middle value", processor.middleSum);
            telemetry.update();
        }

        waitForStart();
    }
}
