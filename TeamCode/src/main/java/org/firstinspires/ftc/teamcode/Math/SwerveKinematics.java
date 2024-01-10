package org.firstinspires.ftc.teamcode.Math;

import static java.lang.Math.PI;

import java.util.ArrayList;
import java.util.HashMap;

public class SwerveKinematics {

    private ArrayList<Double> angles = new ArrayList<>();
    private HashMap<Double, Vector2d> wheelVectors = new HashMap<>();

    public SwerveKinematics(ArrayList<Double> angles){
        this.angles = angles;
        for (double angle: angles) {
            wheelVectors.put(angle, new Vector2d(0,0));
        }
    }

    public void calculate(Vector2d translationalMovement, double rotationalMovement){
        double maxMagnitude = 0;
        for (double angle: angles) {
            calculate(translationalMovement, rotationalMovement, angle);
            maxMagnitude = Math.max(maxMagnitude, getWheelPower(angle));
        }
        if(maxMagnitude > 1){
            for(double angle: angles){
                wheelVectors.get(angle).scale(maxMagnitude);
            }
        }
    }

    private void calculate(Vector2d translationalMovement, double rotationalMovement, double angle){
        Vector2d rotationalVector = new Vector2d(Math.cos(angle + PI/2.0) * rotationalMovement, Math.sin(angle + PI/2.0) * rotationalMovement);
        wheelVectors.put(angle, new Vector2d(translationalMovement.x + rotationalVector.x,
                translationalMovement.y + rotationalVector.y));
    }

    public double getWheelTargetAngle(double wheelAngle){
        return Math.atan2(wheelVectors.get(wheelAngle).y, wheelVectors.get(wheelAngle).x);
    }

    public double getWheelPower(double wheelAngle){
        return wheelVectors.get(wheelAngle).getMagnitude();
    }

    public static class Vector2d{
        public double x,y;

        public Vector2d(double x, double y){
            this.x = x;
            this.y = y;
        }

        public double getMagnitude(){
            return Math.sqrt(x*x + y*y);
        }

        public void scale(double scalar){
            x*=scalar;
            y*=scalar;
        }
    }
}


