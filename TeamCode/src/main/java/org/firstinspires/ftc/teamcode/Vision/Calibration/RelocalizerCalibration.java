package org.firstinspires.ftc.teamcode.Vision.Calibration;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Vision.BlueRelocalizer;
import org.firstinspires.ftc.teamcode.Vision.RedRelocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Disabled
@Config
@Autonomous(name="Relocalization calibration")
public class RelocalizerCalibration extends LinearOpMode {
    VisionPortal portal;
    AprilTagProcessor processor;

    Hardware hardware;

    RedRelocalizer relocalizer;

    @Override
    public void runOpMode() throws InterruptedException {
        processor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .build();

        hardware = new Hardware(hardwareMap);

        relocalizer = new RedRelocalizer(processor, hardware.localizer);

        hardware.startThreads(this);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            relocalizer.update();
            telemetry.addData("Pose", hardware.localizer.getPoseEstimate());
            telemetry.addData("Last tag X", relocalizer.lastX);
            telemetry.addData("Last tag Y", relocalizer.lastY);
            telemetry.addData("Tag distance", relocalizer.distance);
            telemetry.addData("Global tag angle", relocalizer.globalAngle);
            telemetry.addData("Time since detection", (System.nanoTime() - relocalizer.frameAcquisitionTime)/1e9);
            telemetry.update();
        }
    }
}

