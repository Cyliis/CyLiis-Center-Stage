package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.GPose;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTangentHeadingTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Utils.Pose;

import java.util.List;

//@Disabled
@Config
@TeleOp(name = "Sample bezier auto")
public class SampleOpMode extends LinearOpMode {
    Hardware hardware;

    MecanumDrive drive;
    Follower follower;
    Localizer localizer;
    FtcDashboard dash;

    CubicBezierTrajectorySegment segment1 =
            new CubicBezierTrajectorySegment(
                    new Pose(0,0,0),
                    new Pose(19.6, 12, -PI/4.0),
                    new Pose(40, 30, -PI/2.0),
                    new Pose(50, -6,-PI/2.0)
            );
//            new CubicBezierTrajectorySegment(
//                    new Pose(0,0,0),
//                    new Pose(19.6, 4, -PI/4.0),
//                    new Pose(25.5, 8.5, -PI/2.0),
//                    new Pose(27.5, -6,-PI/2.0)
//            );

    Trajectory lol = new TrajectoryBuilder(segment1)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        hardware = new Hardware(hardwareMap);
        localizer = hardware.localizer;

        drive = new MecanumDrive(hardware, hardware.localizer, false);
        follower = new Follower(drive, hardware.localizer);
        follower.setTrajectory(lol);

        waitForStart();

        hardware.startThreads(this);

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();
//            hardware.localizer.update();
            follower.update();
            drive.update();
            telemetry.addData("Hz",1.0/ loopTimer.seconds());
            telemetry.addData("Followed point", follower.currentFollowedPoint);
            telemetry.addData("Pose", hardware.localizer.getPoseEstimate());
            telemetry.addData("Trail size", follower.trail.size());
            telemetry.addData("Mecanum run mode", drive.getRunMode());
            telemetry.addData("Y component of power vector", drive.powerVector.getY());
            loopTimer.reset();
//            follower.draw(dash);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();

            GPose gp = localizer.getPoseEstimate().getGPose();

            follower.trail.add(gp);

            packet.fieldOverlay()
                    .strokeDesiredPath(follower.getTrajectory().getGPoses())
                    .strokeActualPath(follower.trail)
                    .setStroke("#42f54b")
                    .strokeRobot(gp);

            dash.sendTelemetryPacket(packet);

        }
    }
}