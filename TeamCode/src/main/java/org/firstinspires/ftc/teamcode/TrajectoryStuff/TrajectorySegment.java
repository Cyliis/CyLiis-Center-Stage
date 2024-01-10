package org.firstinspires.ftc.teamcode.TrajectoryStuff;

import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

import java.util.ArrayList;

public abstract class TrajectorySegment {
    public static int resolution = 1000;

    public abstract Vector getTangentVelocity(double t);
    public abstract double getCurvature(double t);
    public abstract Pose getPose(double t);
    public abstract double getHeading(double t);
    public abstract ArrayList<Double> getClosePoints(Pose pose);
    public abstract double getDistanceFromPoint(Pose pose, double t);
    public abstract double getLengthAt(double t);
    public abstract double getLength();
}