package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "OpPark")
public class OpPark extends OpMode {

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    //TODO: change the time the robot moves for
    public static double forwardTime = 0.75;
    //TODO: change the power given to the wheels
    public static double power = 1;

    @Override
    public void init() {
        //TODO: change the strings to match your hardware map

        frontLeft = hardwareMap.get(DcMotorEx.class, "ch1");
        frontRight = hardwareMap.get(DcMotorEx.class, "ch0");
        backLeft = hardwareMap.get(DcMotorEx.class, "ch2");
        backRight = hardwareMap.get(DcMotorEx.class, "ch3");

        //TODO: comment/uncomment to match your real orientation

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    ElapsedTime timer = new ElapsedTime();
    boolean timerStarted = false;

    @Override
    public void loop() {
        if(!timerStarted) {
            timer.startTime();
            timer.reset();
            timerStarted = true;
        }

        if(timer.seconds() <= forwardTime){
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}