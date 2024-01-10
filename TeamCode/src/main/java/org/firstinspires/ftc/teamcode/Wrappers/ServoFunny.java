package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoFunny {

    public Servo servo;
    private double position = -1;

    final Object lock = new Object();

    public ServoFunny(Servo servo){
        this.servo = servo;
    }

    public void setPositionAsync(double position){
        synchronized (lock) {
            this.position = position;
        }
    }

    public void updatePositionAsync(){
        synchronized (lock) {
            if(position != -1)
                servo.setPosition(position);
        }
    }
}
