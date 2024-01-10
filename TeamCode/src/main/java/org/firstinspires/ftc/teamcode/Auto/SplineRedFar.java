package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LogicNodes.Nodes.SplineRedFarNodes;
import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Vision.PropDetectionRedFar;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Spline Red Far 🐟")
public class SplineRedFar extends LinearOpMode {
    Hardware hardware;

    MecanumDrive drive;
    Follower follower;
    RobotModules robot;

    SplineRedFarNodes nodes;

    FtcDashboard dash;

//    VisionPortal portal;
//    PropDetectionRedFar processor;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        drive = new MecanumDrive(hardware, hardware.localizer, MecanumDrive.RunMode.PID, false);
        follower = new Follower(drive, hardware.localizer);

        robot = new RobotModules(hardware);

        hardware.startThreads(this);

//        processor = new PropDetectionRedFar();
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(640, 480))
//                .setCamera(BuiltinCameraDirection.BACK)
//                .addProcessor(processor)
//                .enableLiveView(true)
//                .build();

        int detectionCase = 3;

//        while(opModeInInit() && !isStopRequested()){
//
//            detectionCase = processor.detection;
//
//            robot.initUpdate();
//            robot.telemetry(telemetry);
//            telemetry.addData("Detection", processor.detection);
//            telemetry.addLine("RIGHT GRIPPER");
//            telemetry.update();
//        }

        nodes = new SplineRedFarNodes(robot, follower, drive, detectionCase);

//        portal.close();

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();
            nodes.currentNode.run();
            follower.update();
            drive.update();
            robot.update();

            telemetry.addData("Imu angle", drive.getLocalizer().getHeading());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            telemetry.addData("Current node", nodes.currentNode);
            telemetry.addData("Left sensor", hardware.leftSensor.isPixel());
            telemetry.addData("Right sensor", hardware.rightSensor.isPixel());
            robot.telemetry(telemetry);

            loopTimer.reset();

            telemetry.update();
        }

        CoolIMU.imuOffset = -hardware.imu.getHeading();
    }
}
