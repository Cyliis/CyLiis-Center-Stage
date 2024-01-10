package org.firstinspires.ftc.teamcode.Math;

public class LowPassFilter {

    private final double t;
    private double lastValue;

    public LowPassFilter (double t, double initialValue){
        this.t = t;
        this.lastValue = initialValue;
    }

    public double getValue(double rawValue){
        double newValue = lastValue + t * (rawValue - lastValue);
        this.lastValue = newValue;
        return newValue;
    }
}
