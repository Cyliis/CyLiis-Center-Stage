package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.GamepadControllers.BuruSebiGamepadControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.Pose;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp 🐟")
public class TeleOp extends LinearOpMode {

    FtcDashboard dash;

    Hardware hardware;

    MecanumDrive drive;
    RobotModules robotModules;

    BuruSebiGamepadControl gamepadControl;
    BuruDriveTrainControl driveTrainControl;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        drive = new MecanumDrive(hardware, hardware.localizer, true);

        robotModules = new RobotModules(hardware);

        gamepadControl = new BuruSebiGamepadControl(robotModules, gamepad1, gamepad2);
        driveTrainControl = new BuruDriveTrainControl(gamepad1, drive);

        hardware.startThreads(this);

        while(opModeInInit() && !isStopRequested()){
            robotModules.initUpdate();
            robotModules.telemetry(telemetry);
            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.update();

            gamepadControl.update();
            driveTrainControl.update();

//            hardware.localizer.update();
            drive.update();

            robotModules.update();

            robotModules.telemetry(telemetry);

            telemetry.addData("Imu angle", drive.getLocalizer().getHeading());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}
