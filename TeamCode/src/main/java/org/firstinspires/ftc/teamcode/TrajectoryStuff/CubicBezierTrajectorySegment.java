package org.firstinspires.ftc.teamcode.TrajectoryStuff;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Math.Polynomial;
import org.firstinspires.ftc.teamcode.Math.SixthDegreePolynomialMagic;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

import java.util.ArrayList;

@Config
public class CubicBezierTrajectorySegment extends TrajectorySegment {

    private final Pose startPoint, endPoint;
    private final Pose controlPoint0, controlPoint1;

    public Polynomial pX, pY, pH;
    public Polynomial pdX, pdY;
    public Polynomial pd2X, pd2Y;

    private double length = 0;

    public static int resolution = 1000;

    private ArrayList<Double> lengthArray = new ArrayList<>();

    public CubicBezierTrajectorySegment(Pose startPoint, Pose controlPoint0, Pose controlPoint1, Pose endPoint){
        this.startPoint = startPoint;
        this.controlPoint0 = controlPoint0;
        this.controlPoint1 = controlPoint1;
        this.endPoint = endPoint;
        init();
    }

    public CubicBezierTrajectorySegment(CubicBezierTrajectorySegment previousCurve, Pose controlPoint1, Pose endPoint){
        this.startPoint = previousCurve.endPoint;
        this.controlPoint0 = new Pose(previousCurve.endPoint.getX() + (previousCurve.endPoint.getX() - previousCurve.controlPoint1.getX()),
                previousCurve.endPoint.getY() + (previousCurve.endPoint.getY() - previousCurve.controlPoint1.getY()), previousCurve.controlPoint1.getHeading());
        this.controlPoint1 = controlPoint1;
        this.endPoint = endPoint;
        init();
    }

    private void init(){
        double xa = endPoint.getX() - 3.0 * controlPoint1.getX() + 3.0 * controlPoint0.getX() - startPoint.getX();
        double xb = 3.0 * (controlPoint1.getX() - 2.0 * controlPoint0.getX() + startPoint.getX());
        double xc = 3.0 * (controlPoint0.getX() - startPoint.getX());
        double xd = startPoint.getX();

        pX = new Polynomial(xa, xb, xc, xd);
        pdX = pX.getDerivative();
        pd2X = pdX.getDerivative();

        double ya = endPoint.getY() - 3.0 * controlPoint1.getY() + 3.0 * controlPoint0.getY() - startPoint.getY();
        double yb = 3.0 * (controlPoint1.getY() - 2.0 * controlPoint0.getY() + startPoint.getY());
        double yc = 3.0 * (controlPoint0.getY() - startPoint.getY());
        double yd = startPoint.getY();

        pY = new Polynomial(ya, yb, yc, yd);
        pdY = pY.getDerivative();
        pd2Y = pdY.getDerivative();

        double ha = endPoint.getHeading() - 3.0 * controlPoint1.getHeading() + 3.0 * controlPoint0.getHeading() - startPoint.getHeading();
        double hb = 3.0 * (controlPoint1.getHeading() - 2.0 * controlPoint0.getHeading() + startPoint.getHeading());
        double hc = 3.0 * (controlPoint0.getHeading() - startPoint.getHeading());
        double hd = startPoint.getHeading();

        pH = new Polynomial(ha, hb, hc, hd);

        computeLength();
    }

    private void computeLength(){
        double dx, dy;
        double dt = 1.0/(double)resolution;
        for(double d = 0; d <= 1; d += dt) {
            lengthArray.add(length);

            dx = pdX.evaluate(d);
            dy = pdY.evaluate(d);

            length+=Math.sqrt(dx*dx + dy*dy)*dt;
        }
    }

    public Vector getTangentVelocity(double t) {
        Vector ans = new Vector(pdX.evaluate(t), pdY.evaluate(t)).scaledToMagnitude(1);
        return ans;
    }

    public double getCurvature(double t) {
        double dx = pdX.evaluate(t);
        double dy = pdY.evaluate(t);
        double d2x = pd2X.evaluate(t);
        double d2y = pd2Y.evaluate(t);

        return (dx * d2y - d2x * dy) /
                (Math.sqrt(Math.pow(dx * dx  + dy * dy,3)));
    }

    public Pose getPose(double t) {
        if(t==1) return endPoint;
        return new Pose(pX.evaluate(t), pY.evaluate(t), pH.evaluate(t));
    }

    public double getHeading(double t) {
        return getPose(t).getHeading();
    }

    public Polynomial distancePolynomial;

    @Override
    public ArrayList<Double> getClosePoints(Pose pose) {
        double px[]={pX.getCoefficient(0),pX.getCoefficient(1),pX.getCoefficient(2), pX.getCoefficient(3)};
        double py[]={pY.getCoefficient(0),pY.getCoefficient(1),pY.getCoefficient(2), pY.getCoefficient(3)};

        double ax, bx, cx, dx;
        double ay, by, cy, dy;

        ax = px[3];
        bx = px[2];
        cx = px[1];
        dx = px[0] - pose.getX();

        ay = py[3];
        by = py[2];
        cy = py[1];
        dy = py[0] - pose.getY();

        double a,b,c,d,e,f,g;

        a=ax*ax + ay*ay;
        b=2*ax*bx + 2*ay*by;
        c=2*ax*cx+bx*bx + 2*ay*cy+by*by;
        d=2*ax*dx+2*bx*cx + 2*ay*dy+2*by*cy;
        e=2*bx*dx+cx*cx+ 2*by*dy+cy*cy;
        f=2*cx*dx + 2*cy*dy;
        g=dx*dx + dy*dy;

        distancePolynomial = new Polynomial(a,b,c,d,e,f,g);

        return SixthDegreePolynomialMagic.getAllValleys(distancePolynomial,0,1);
    }

    public double getDistanceFromPoint(Pose pose, double t) {
        return new Vector(pose.getX(), pose.getY()).plus(new Vector(-getPose(t).getX(), -getPose(t).getY())).getMagnitude();
    }

    public double getLengthAt(double t) {
        int index = (int)(t * (double)resolution);
        index = Math.min(resolution - 1, Math.max(0,index));
        return lengthArray.get(index) + (lengthArray.get(Math.min(resolution - 1, index + 1)) - lengthArray.get(index)) * 0.5;
    }

    public double getLength(){
        return length;
    }


}