package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Localizer;

public class CoolIMU {

    private final Object imuLock1 = new Object();
    private final Object imuLock2 = new Object();
    private final IMU imu;
    public static double imuAngle = 0;
    public static double imuVelocity = 0;
    public static double imuOffset = 0;

    public CoolIMU(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        imu.resetYaw();
    }

    public void startIMUThread(LinearOpMode opMode, com.acmerobotics.roadrunner.localization.Localizer localizer) {
        new Thread(() -> {
            imu.resetYaw();
            while ((opMode.opModeIsActive() && !opMode.isStopRequested()) || (opMode.opModeInInit() && !opMode.isStopRequested())) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                synchronized (imuLock1) {
                    imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    localizer.update();
                }
                synchronized (imuLock2){
                    imuVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
            }
        }).start();
    }

    public void startIMUThread(LinearOpMode opMode, Localizer localizer) {
        new Thread(() -> {
            imu.resetYaw();
            while ((opMode.opModeIsActive() && !opMode.isStopRequested()) || (opMode.opModeInInit() && !opMode.isStopRequested())) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                synchronized (imuLock1) {
                    imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    localizer.update();
                }
                synchronized (imuLock2){
                    imuVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
            }
        }).start();
    }

    public synchronized void resetImu(){
        imu.resetDeviceConfigurationForOpMode();
        imuOffset = 0;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public double getHeading() {
        return imuAngle - imuOffset;
    }
    public double getVelocity() {
        return imuVelocity;
    }

}