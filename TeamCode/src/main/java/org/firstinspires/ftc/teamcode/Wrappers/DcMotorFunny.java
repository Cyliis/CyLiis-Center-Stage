package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorFunny {

    public DcMotorEx motor;
    private double power;

    final Object lock = new Object();

    public DcMotorFunny(DcMotorEx motor){
        this.motor = motor;
    }

    public void setPowerAsync(double power){
        synchronized (lock) {
            this.power = power;
        }
    }

    public void updatePowerAsync(){
        synchronized (lock) {
            motor.setPower(power);
        }
    }
}
