package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.RightPixelColorSensor;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Modules.LeftPixelColorSensor;

public class Hardware {
    public HardwareMap hardwareMap;

    public DcMotorEx mch0, mch1, mch2, mch3;
    public DcMotorEx meh0, meh1, meh2, meh3;

    public Encoder ech0, ech1, ech2, ech3;
    public Encoder eeh0, eeh1, eeh2, eeh3;

    public Servo sch0, sch1, sch2, sch3, sch4, sch5;
    public Servo seh0, seh1, seh2, seh3, seh4, seh5;

    public CoolIMU imu;

    public LynxModule chub, ehub;

    public Localizer localizer;

    public LeftPixelColorSensor leftSensor;
    public RightPixelColorSensor rightSensor;

    public VoltageSensor voltageSensor;

    public Hardware(HardwareMap hm){
        this.hardwareMap = hm;

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

//        chub = hardwareMap.getAll(LynxModule.class).get(0).isParent() ?
//                hardwareMap.getAll(LynxModule.class).get(0) : hardwareMap.getAll(LynxModule.class).get(1);
//
//        ehub = hardwareMap.getAll(LynxModule.class).get(1).isParent() ?
//                hardwareMap.getAll(LynxModule.class).get(0) : hardwareMap.getAll(LynxModule.class).get(1);

        mch0 = hm.get(DcMotorEx.class, "ch0");
        mch1 = hm.get(DcMotorEx.class, "ch1");
        mch2 = hm.get(DcMotorEx.class, "ch2");
        mch3 = hm.get(DcMotorEx.class, "ch3");

        meh0 = hm.get(DcMotorEx.class, "eh0");
        meh1 = hm.get(DcMotorEx.class, "eh1");
        meh2 = hm.get(DcMotorEx.class, "eh2");
        meh3 = hm.get(DcMotorEx.class, "eh3");

        ech0 = new Encoder(mch0);
        ech1 = new Encoder(mch1);
        ech2 = new Encoder(mch2);
        ech3 = new Encoder(mch3);

//        eeh0 = new Encoder(meh0);
//        eeh1 = new Encoder(meh1);
//        eeh2 = new Encoder(meh2);
//        eeh3 = new Encoder(meh3);

        sch0 = hm.get(Servo.class, "sch0");
        sch1 = hm.get(Servo.class, "sch1");
        sch2 = hm.get(Servo.class, "sch2");
        sch3 = hm.get(Servo.class, "sch3");
        sch4 = hm.get(Servo.class, "sch4");
        sch5 = hm.get(Servo.class, "sch5");

        seh0 = hm.get(Servo.class, "seh0");
        seh1 = hm.get(Servo.class, "seh1");
        seh2 = hm.get(Servo.class, "seh2");
        seh3 = hm.get(Servo.class, "seh3");
        seh4 = hm.get(Servo.class, "seh4");
        seh5 = hm.get(Servo.class, "seh5");

        leftSensor = new LeftPixelColorSensor(hm.get(ColorRangeSensor.class, "leftSensor"));
        rightSensor = new RightPixelColorSensor(hm.get(ColorRangeSensor.class, "rightSensor"));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        imu = new CoolIMU(hm);

        localizer = new Localizer(this);
    }

    public void startThreads(LinearOpMode opMode, Localizer localizer){
        imu.startIMUThread(opMode, localizer);
    }

    public void startThreads(LinearOpMode opMode, com.acmerobotics.roadrunner.localization.Localizer localizer){
        imu.startIMUThread(opMode, localizer);
    }

    public void update(){
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
    }


}
