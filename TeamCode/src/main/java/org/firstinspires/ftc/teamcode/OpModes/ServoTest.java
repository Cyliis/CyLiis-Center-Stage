package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends OpMode {

    Servo a,b,c,d,e,f,g;

    @Override
    public void init() {
        a = hardwareMap.get(Servo.class, "0");
        b = hardwareMap.get(Servo.class, "1");
        c = hardwareMap.get(Servo.class, "2");
        d = hardwareMap.get(Servo.class, "3");
        e = hardwareMap.get(Servo.class, "4");
        f = hardwareMap.get(Servo.class, "5");
    }

    @Override
    public void loop() {
        a.setPosition((System.nanoTime()/1e9)%1.0);
        b.setPosition((System.nanoTime()/1e9)%1.0);
        c.setPosition((System.nanoTime()/1e9)%1.0);
        d.setPosition((System.nanoTime()/1e9)%1.0);
        e.setPosition((System.nanoTime()/1e9)%1.0);
        f.setPosition((System.nanoTime()/1e9)%1.0);
    }
}
