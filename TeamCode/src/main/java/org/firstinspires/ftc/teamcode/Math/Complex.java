package org.firstinspires.ftc.teamcode.Math;

import org.firstinspires.ftc.teamcode.Utils.Vector;

public class Complex {
    public double real, imag;
    public Complex(double real, double imag){
        this.real = real;
        this.imag = imag;
    }
    public Complex conj(){
        return new Complex(real, -imag);
    }
    public Complex mult(Complex other){
        Vector a = new Vector(real, imag);
        Vector b = new Vector(other.real, other.imag);
        Vector c = Vector.fromAngleAndMagnitude(Math.atan2(a.getY(), a.getX()) + Math.atan2(b.getY(), b.getX()),a.getMagnitude()*b.getMagnitude());
        return new Complex(c.getX(), c.getY());
    }
    public Complex add(Complex other){
        return new Complex(real + other.real, imag + other.imag);
    }
    public static Complex exp(Complex a){
        Complex real = new Complex(Math.exp(a.real),0);
        Complex imag = new Complex(Math.cos(a.imag), Math.sin(a.imag));
        return real.mult(imag);
    }
    @Override
    public String toString(){
        return "(" + String.valueOf(real) + "," + String.valueOf(imag) + ")";
    }
}
