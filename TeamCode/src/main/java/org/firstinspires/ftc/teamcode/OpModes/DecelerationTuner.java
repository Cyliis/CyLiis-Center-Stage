package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruSebiGamepadControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Disabled
@Config
@TeleOp(name = "Deceleration Tuner")
public class DecelerationTuner extends LinearOpMode {

    FtcDashboard dash;

    Hardware hardware;

    MecanumDrive drive;

    ElapsedTime timer = new ElapsedTime();

    public static double accelerationTime = 2;
    private boolean stopped = false;
    private double velocityAtStop;

    private double deceleration;
    private double deltaTime;

    int step = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        drive = new MecanumDrive(hardware, hardware.localizer, false);
        hardware.startThreads(this, drive.getLocalizer());

        waitForStart();

        timer.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {

            hardware.update();

            switch (step){
                case 0:
                    if(timer.seconds() <= accelerationTime) {
                        drive.setTargetVector(new Vector(0,1,0));
                    }
                    else{
                        step++;
                        timer.reset();
                        velocityAtStop = hardware.localizer.getVelocity().getY();
                        drive.setTargetVector(new Vector());
                    }
                    break;
                case 1:
                    if(hardware.localizer.getVelocity().getMagnitude()<=0.1)
                    {
                        step++;
                        deltaTime = timer.seconds();
                        deceleration = velocityAtStop / deltaTime;
                    }
                    break;
            }

//            hardware.localizer.update();
            drive.update();

            telemetry.addData("pose", hardware.localizer.getPoseEstimate());
            telemetry.addData("Step", step);
            telemetry.addData("Timer", timer.seconds());
            telemetry.addData("Deceleration", deceleration);
            telemetry.addData("Delta time", deltaTime);
            telemetry.addData("Velocity at stop", velocityAtStop);
            telemetry.addData("Time since stop", timer.seconds());
            telemetry.addData("Stopped", stopped);
            telemetry.addData("Velocity x", drive.getLocalizer().getVelocity().getX());
            telemetry.addData("Velocity y", drive.getLocalizer().getVelocity().getY());
            telemetry.addData("Imu angle", drive.getLocalizer().getHeading());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}