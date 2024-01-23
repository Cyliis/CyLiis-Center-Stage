package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.LowPassFilter;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.FunnyLocalizer;

import java.util.ArrayList;

@Config
public class Localizer implements IRobotModule {

    public static boolean ENABLED = true;

    protected Pose pose;
    public com.acmerobotics.roadrunner.localization.Localizer localizer;
    public CoolIMU imu;

    public Localizer(Hardware hardware, Pose initialPose) {
        this.pose = initialPose;
        this.imu = hardware.imu;
        this.localizer = new FunnyLocalizer(hardware);
        localizer.setPoseEstimate(new Pose2d(initialPose.getX(), initialPose.getY(), initialPose.getHeading()));
    }

    public Localizer(Hardware hardware) {
        this.pose = new Pose();
        this.imu = hardware.imu;
        this.localizer = new FunnyLocalizer(hardware);
        localizer.setPoseEstimate(new Pose2d());
    }

    public void setPose(Pose pose) {
        localizer.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), pose.getHeading()));
        this.pose = pose;
    }

    public Pose getPoseEstimate() {
        return pose;
    }

    public Pose getPredictedPoseEstimate(){
        return new Pose(pose.getX() + glideDelta.getX(), pose.getY() + glideDelta.getY(), pose.getHeading());
    }

    public double getHeading(){
        return pose.getHeading();
    }

    private Vector velocity = new Vector();
    public Vector glideDelta = new Vector();

    public static double filterParameter = 0.8;
    private final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0),
            yVelocityFilter = new LowPassFilter(filterParameter, 0);

    public static double xDeceleration = 45, yDeceleration = 120;

    public Vector getVelocity(){
        return velocity;
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        localizer.update();
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
        velocity = new Vector(xVelocityFilter.getValue(localizer.getPoseVelocity().getX()), yVelocityFilter.getValue(localizer.getPoseVelocity().getY()));
        Vector predictedGlideVector = new Vector(Math.signum(velocity.getX()) * velocity.getX() * velocity.getX() / (2.0 * xDeceleration), Math.signum(velocity.getY()) * velocity.getY() * velocity.getY() / (2.0 * yDeceleration));
        glideDelta = Vector.rotateBy(predictedGlideVector, -pose.getHeading());
    }
}