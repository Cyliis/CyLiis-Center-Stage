package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Pose {
    private final double x,y,heading;
    private final double tolerance;
    public static double defaultTolerance = 0.5;

    public Pose(double x, double y, double heading, double tolerance){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.tolerance = tolerance;
    }

    public Pose(double x, double y, double heading){
        this(x, y, heading,defaultTolerance);
    }

    public Pose(double x, double y){
        this(x, y, 0,defaultTolerance);
    }

    public Pose(){
        this(0,0,0,defaultTolerance);
    }

    public double getX(){
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getTolerance(){
        return tolerance;
    }

    public double getDistance(Pose other){
        return Math.sqrt((other.getX() - x)*(other.getX() - x) + (other.getY() - y)*(other.getY() - y));
    }

    public boolean isReached(Pose other){
        return getDistance(other) <= tolerance;
    }

    public Pose plus(Pose other){
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    @Override
    public String toString(){
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(heading);
    }
}
