package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.Pose;

@Disabled
@Config
@TeleOp(name = "Correction PID Tuning")
public class CorrectionPIDTuning extends LinearOpMode {

    Hardware hardware;

    MecanumDrive drive;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        drive = new MecanumDrive(hardware, hardware.localizer, MecanumDrive.RunMode.PID, false);

        hardware.startThreads(this);

        waitForStart();

        drive.setTargetPose(new Pose());

        while(opModeIsActive() && !isStopRequested()){
            hardware.update();
//            hardware.localizer.update();
            drive.update();

            telemetry.addData("Pose X", hardware.localizer.getPoseEstimate().getX());
            telemetry.addData("Pose Y", hardware.localizer.getPoseEstimate().getY());
            telemetry.addData("Pose Heading", hardware.localizer.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
