package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LogicNodes.Nodes.RedFarNodes;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Vision.RedRelocalizer;
import org.firstinspires.ftc.teamcode.Vision.PropDetectionRedFar;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Better Red Far 🐟")
public class RedFar extends LinearOpMode {
    Hardware hardware;

    MecanumDrive drive;
    RobotModules robot;

    RedFarNodes nodes;

    FtcDashboard dash;

    VisionPortal portal;
    PropDetectionRedFar processor;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        drive = new MecanumDrive(hardware, hardware.localizer, MecanumDrive.RunMode.PID, false);
        robot = new RobotModules(hardware);

        hardware.startThreads(this);

        processor = new PropDetectionRedFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();

        int detectionCase = 1;

        while(opModeInInit() && !isStopRequested()){

            detectionCase = processor.detection;
//            detectionCase = 1;

            robot.initUpdate();
            robot.telemetry(telemetry);
            telemetry.addData("Detection", processor.detection);
            telemetry.addLine("RIGHT GRIPPER");
            telemetry.update();
        }

        nodes = new RedFarNodes(drive, robot, detectionCase);
        nodes.currentNode = nodes.start;

        portal.close();

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();
//            hardware.localizer.update();
            nodes.currentNode.run();
            drive.update();
            robot.update();

            telemetry.addData("Imu angle", drive.getLocalizer().getHeading());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            telemetry.addData("Current node", nodes.currentNode);
            telemetry.addData("Left sensor", hardware.leftSensor.isPixel());
//            telemetry.addData("Diff", ((drive.getTargetPose().getHeading() - ((hardware.localizer.getHeading()%(2.0*PI) - Math.signum(hardware.localizer.getHeading()) * PI * 2.0) % (2.0* PI)) )%(2.0*PI)));
            telemetry.addData("Right sensor", hardware.rightSensor.isPixel());
//            telemetry.addData("Diff", drive.diff);
            telemetry.addData("Pose", hardware.localizer.getPoseEstimate());
            robot.telemetry(telemetry);

            loopTimer.reset();

            telemetry.update();
        }
    }
}
