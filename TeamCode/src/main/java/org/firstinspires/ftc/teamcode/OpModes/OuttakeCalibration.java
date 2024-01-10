package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.OuttakeArm;
import org.firstinspires.ftc.teamcode.Modules.Pitch;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Disabled
@TeleOp(name = "Outtake Calibration")
public class OuttakeCalibration extends LinearOpMode {

    Hardware hardware;
    Lift lift;
    OuttakeArm arm;
    Pitch pitch;

    StickyGamepad gamepad;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap);

        gamepad = new StickyGamepad(gamepad1);

        lift = new Lift(hardware, Lift.State.DOWN);
        arm = new OuttakeArm(hardware, OuttakeArm.State.INTAKE);
        pitch = new Pitch(hardware, Pitch.State.INTAKE);

        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware.startThreads(this);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            hardware.update();
            gamepad.update();

            if (gamepad1.dpad_up) {
                if (gamepad.x) lift.setState(Lift.State.GOING_UP);
                if (gamepad.a) lift.setState(Lift.State.GOING_DOWN);
                if (gamepad.b) lift.setState(Lift.State.GOING_PASSTHROUGH);
            }
            if (gamepad1.dpad_down) {
                if (gamepad.a) pitch.setState(Pitch.State.GOING_OUTTAKE);
                if (gamepad.x) pitch.setState(Pitch.State.GOING_INTAKE);
            }
            if (gamepad1.dpad_left) {
                if (gamepad.x) arm.setState(OuttakeArm.State.GOING_INTAKE);
                if (gamepad.a) arm.setState(OuttakeArm.State.GOING_OUTTAKE);
            }
            lift.update();
            arm.update();
            pitch.update();

            telemetry.addData("Lift target pos", lift.getState().position);
            telemetry.addData("Lift current pos", lift.encoder.getCurrentPosition());
            telemetry.addData("Pitch pos", pitch.getState().position);
            telemetry.addData("Arm pos", arm.getState().position);

            telemetry.addData("Lift state", lift.getState());
            telemetry.addData("Pitch state", pitch.getState());
            telemetry.addData("Arm state", arm.getState());

            telemetry.update();
        }
    }
}
