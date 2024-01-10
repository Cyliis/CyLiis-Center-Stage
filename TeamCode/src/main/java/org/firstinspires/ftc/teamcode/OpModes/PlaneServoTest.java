package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "lol")
public class PlaneServoTest extends OpMode {
    Servo servo;
    double pos = 0.5;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "seh1");
        servo.setPosition(pos);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) pos += 0.001;
        if(gamepad1.dpad_down) pos -= 0.001;
        servo.setPosition(pos);
        telemetry.addData("Pos", pos);
        telemetry.update();
    }
}
