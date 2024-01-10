package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.DropDown;
import org.firstinspires.ftc.teamcode.Modules.LeftGripper;
import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.OuttakeArm;
import org.firstinspires.ftc.teamcode.Modules.Pitch;
import org.firstinspires.ftc.teamcode.Modules.RightGripper;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Disabled
@TeleOp(name = "Intake Calibration")
public class IntakeCalibration extends LinearOpMode {

    Hardware hardware;
    DropDown dropDown;
    LeftGripper leftGripper;
    RightGripper rightGripper;

    StickyGamepad gamepad;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap);

        gamepad = new StickyGamepad(gamepad1);

        dropDown = new DropDown(hardware, DropDown.State.UP);
        leftGripper = new LeftGripper(hardware, LeftGripper.State.CLOSED);
        rightGripper = new RightGripper(hardware, RightGripper.State.CLOSED);

        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware.startThreads(this);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            hardware.update();
            gamepad.update();
            if(gamepad1.dpad_up){
                if(gamepad.x) dropDown.setState(DropDown.State.GOING_UP);
                if(gamepad.a) dropDown.setState(DropDown.State.GOING_INTAKE);
            }
            if(gamepad1.dpad_right){
                if(gamepad.x) rightGripper.setState(RightGripper.State.CLOSING);
                if(gamepad.a) rightGripper.setState(RightGripper.State.OPENING);
            }
            if(gamepad1.dpad_left){
                if(gamepad.x) leftGripper.setState(LeftGripper.State.CLOSING);
                if(gamepad.a) leftGripper.setState(LeftGripper.State.OPENING);
            }
            dropDown.update();
            rightGripper.update();
            leftGripper.update();

            telemetry.addData("Drop down pos", dropDown.getState().position);
            telemetry.addData("Left gripper pos", leftGripper.getState().position);
            telemetry.addData("Right gripper pos", rightGripper.getState().position);

            telemetry.addData("Drop down state", dropDown.getState());
            telemetry.addData("Left gripper state", leftGripper.getState());
            telemetry.addData("Right gripper state", rightGripper.getState());

            telemetry.update();
        }
    }
}
