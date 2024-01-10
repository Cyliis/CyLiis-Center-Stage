package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Utils.EasyPrecision.precision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@TeleOp(name="kStatic calibration")
public class kStaticCalibration extends LinearOpMode {

    Hardware hardware;

    MecanumDrive drive;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        drive = new MecanumDrive(hardware, hardware.localizer,false);

        hardware.startThreads(this);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            hardware.update();

//            hardware.localizer.update();

            drive.setTargetVector(new Vector(0.0001, 0, 0));

            drive.update();

            Pose estimate = hardware.localizer.getPoseEstimate();
            Pose predicted = hardware.localizer.getPredictedPoseEstimate();

//            telemetry.addData("Pose","("+precision(estimate.getX())+","+precision(estimate.getY())+","+precision(estimate.getHeading())+")");
//            telemetry.addData("Predicted Pose","("+precision(predicted.getX())+","+precision(predicted.getY())+","+precision(predicted.getHeading())+")");

//            telemetry.addData("Pose X", hardware.localizer.getPoseEstimate().getX());
//            telemetry.addData("Pose Y", hardware.localizer.getPoseEstimate().getY());
//            telemetry.addData("Predicted Pose X", hardware.localizer.getPredictedPoseEstimate().getX());
//            telemetry.addData("Predicted Pose Y", hardware.localizer.getPredictedPoseEstimate().getY());
//            telemetry.addData("Pose Heading", hardware.localizer.getPoseEstimate().getHeading());
//            telemetry.addData("velocity", drive.getLocalizer().getVelocity());
//            telemetry.addData("predicted glide X", drive.getLocalizer().glideDelta.getX());
//            telemetry.addData("predicted glide Y", drive.getLocalizer().glideDelta.getY());
//            telemetry.addData("velocity X", drive.getLocalizer().localizer.getPoseVelocity().getX());
//            telemetry.addData("velocity Y", drive.getLocalizer().localizer.getPoseVelocity().getY());

            telemetry.addData("Voltage", drive.voltage);
            telemetry.update();
        }
    }
}
