package org.firstinspires.ftc.teamcode.Math;

import java.util.ArrayList;

public class Polynomial {
    private ArrayList<Double> coefficients = new ArrayList<>();

    //highest grade coefficient first
    public Polynomial(double... coefficients){
        for(double coefficient: coefficients){
            this.coefficients.add(coefficient);
        }
    }

    public Polynomial(ArrayList<Double> coefficients){
        this.coefficients = coefficients;
    }

    public double getCoefficient(int grade){
        if(grade > coefficients.size() - 1) return 0;
        return coefficients.get(coefficients.size() - 1 - grade);
    }

    public Polynomial getDerivative(){
        ArrayList<Double> derivativeCoefficients = new ArrayList<>();
        int grade = coefficients.size() - 1;
        for(int i = 0; i < grade; i++){
            derivativeCoefficients.add(coefficients.get(i) * (grade-i));
        }
        return new Polynomial(derivativeCoefficients);
    }

    public double evaluate(double t){
        if(coefficients.size() == 0) return 0;
        double ans = 0;
        for(int i = 0;i<coefficients.size();i++){
            ans = ans + coefficients.get(i);
            if(i != coefficients.size() - 1) ans*=t;
        }
        return ans;
    }

    public int getGrade(){
        return coefficients.size()-1;
    }

    public ArrayList<Double> getCoefficients(){
        return coefficients;
    }
}
