package org.firstinspires.ftc.teamcode.Utils;

public class EasyPrecision {
    public static double precision(double x){
        return precision(x,2);
    }
    public static double precision(double x,int decimal){
        return Math.floor(x*Math.pow(10,decimal))/Math.pow(10,decimal);
    }
}
