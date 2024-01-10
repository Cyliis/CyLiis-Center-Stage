package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Disabled
@TeleOp(name = "Color sensor calibration")
public class ColorSensorCalibration extends LinearOpMode {

    Hardware hardware;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);
        
        hardware.startThreads(this);

        waitForStart();
        
        while(opModeIsActive() && !isStopRequested()){
            hardware.update();
            
            telemetry.addData("Left sensor reading", hardware.leftSensor);
            telemetry.addData("Left", hardware.leftSensor.isPixelUpdate());
            telemetry.addData("Right sensor reading", hardware.rightSensor);
            telemetry.addData("Right", hardware.rightSensor.isPixelUpdate());
            telemetry.update();
        }
    }
}
