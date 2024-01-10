package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.PIDCoefficients;

import java.util.ArrayDeque;
import java.util.Deque;

public class PIDController {
    private final PIDCoefficients pid;

    public PIDController(){
        this(new PIDCoefficients(0,0,0));
    }
    public PIDController(PIDCoefficients pid){
        this.pid = pid;
        timer.startTime();
        timer.reset();
    }

    public double integralBound = 0.1;

    private static class IntegralPart{
        public ElapsedTime timer = new ElapsedTime();
        public double value = 0;
        public IntegralPart(double value){
            this.value = value;
            timer.startTime();
            timer.reset();
        }
    }

    private final Deque<IntegralPart> integralParts = new ArrayDeque<>();
    public double integralSum = 0;

    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public double temp = 0;

    public double calculate(double error){
        double ans = error * pid.kP;

        integralSum += error*timer.seconds();
        integralParts.addFirst(new IntegralPart(error));
        temp = integralParts.getLast().timer.seconds();
        while (integralParts.getLast().timer.seconds() > integralBound){
            IntegralPart temp = integralParts.getLast();
            integralParts.removeLast();
            integralSum -= temp.value*(temp.timer.seconds() - integralParts.getLast().timer.seconds());
        }
        ans += integralSum * pid.kI;

        ans += pid.kD * (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return ans;
    }
}
