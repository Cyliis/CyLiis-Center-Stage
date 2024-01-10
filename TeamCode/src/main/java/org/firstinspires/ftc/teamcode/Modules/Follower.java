package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.GPose;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Math.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

import java.util.ArrayList;

@Config
public class Follower implements IRobotModule {

    public static boolean ENABLED = true;

    public static double followingCoefficient = 1, correctionCoefficient = 1, centripetalCorrectionCoefficient = 0.006, headingPIDCoefficient = 1;

    private final MecanumDrive drive;
    private final Localizer localizer;

    private Trajectory trajectory;
    public double currentFollowedPoint = 0.001;
    public double predictedFollowedPoint = 0.001;

    private final PIDController headingPIDController = new PIDController(0,0,0);

    private boolean pid = false;

    public static PIDCoefficients correctionPIDCoefficients = new PIDCoefficients(0,0,0);
    private final PIDController correctionPID = new PIDController(0,0,0);

    public double velocityMultiplier = 1;

    public static double jerk = 8, maxAcceleration = 2;
    public AsymmetricMotionProfile velocityProfile = new AsymmetricMotionProfile(maxAcceleration, jerk, jerk);

    public Follower(MecanumDrive drive, Localizer localizer){
        this.drive = drive;
        this.localizer = localizer;
    }

    public void setTrajectory(Trajectory newTrajectory){
        setTrajectory(newTrajectory, 1);
    }

    public void setTrajectory(Trajectory newTrajectory, double maxPower){
        this.trajectory = newTrajectory;
        this.pid = false;
        this.drive.setRunMode(MecanumDrive.RunMode.Vector);
        this.currentFollowedPoint = 0.001;
        this.velocityMultiplier = maxPower;
        velocityProfile.setMotion(0,1,0);
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }

    public Vector tangentVelocityVector = new Vector();
    public Vector predictedTangentVelocityVector = new Vector();
    public Vector driveVector = new Vector();
    public Vector correctingVector = new Vector();

    public double velocity = 0;

    @Override
    public void update() {
        if(!ENABLED) return;

        if(trajectory == null) return;
        if(pid) return;

        if(trajectory.getLength() - trajectory.getLengthAt(currentFollowedPoint) <= drive.getLocalizer().glideDelta.getMagnitude()){
            pid = true;
            drive.setRunMode(MecanumDrive.RunMode.PID);
            drive.setTargetPose(trajectory.getPose(1));
        }

        velocity = localizer.getVelocity().getMagnitude();

        Pose currentPose = localizer.getPoseEstimate();

        trail.add(currentPose.getGPose());

        Pose predictedPose = localizer.getPredictedPoseEstimate();

        currentFollowedPoint = trajectory.getFollowedPoint(currentPose, currentFollowedPoint);
        predictedFollowedPoint = trajectory.getFollowedPoint(predictedPose, currentFollowedPoint);

        tangentVelocityVector = trajectory.getTangentVelocity(currentFollowedPoint)
                .scaledToMagnitude(velocityProfile.getPosition())
//                .scaledToMagnitude(1)
                .scaledBy(velocityMultiplier)
                .scaledBy(followingCoefficient);
        predictedTangentVelocityVector = trajectory.getTangentVelocity(predictedFollowedPoint).scaledBy(followingCoefficient);

        Pose trajectoryPose = trajectory.getPose(currentFollowedPoint);

        correctionPIDCoefficients = MecanumDrive.translationalPID;
        correctionPID.setPID(correctionPIDCoefficients.p, correctionPIDCoefficients.i, correctionPIDCoefficients.d);

        correctingVector = new Vector(trajectoryPose.getX() - currentPose.getX(), trajectoryPose.getY() - currentPose.getY(), 0);
        double correctionPower = correctionPID.calculate(-correctingVector.getMagnitude(),0);
        correctingVector.scaleToMagnitude(correctionPower);
        correctingVector.scaleBy(correctionCoefficient);

        Vector centripetalCorrectionVector = new Vector(Math.cos(Math.atan2(predictedTangentVelocityVector.getY(), predictedTangentVelocityVector.getX()) + PI/2.0),
                Math.sin(Math.atan2(predictedTangentVelocityVector.getY(), predictedTangentVelocityVector.getX()) + PI/2.0))
                .scaledBy(trajectory.getCurvature(predictedFollowedPoint) * centripetalCorrectionCoefficient)
                .scaledBy(velocity * velocity);


        headingPIDController.setPID(MecanumDrive.headingPID.p, MecanumDrive.headingPID.i, MecanumDrive.headingPID.d);
        double headingDelta = trajectoryPose.getHeading() - currentPose.getHeading();
        headingDelta%=2.0 * PI;
        if(headingDelta > PI) headingDelta -= 2.0*PI;
        if(headingDelta < -PI) headingDelta += 2.0*PI;

        Vector turningVector = new Vector(0, 0, headingPIDController.calculate(-headingDelta, 0)).scaledBy(headingPIDCoefficient);

        driveVector = tangentVelocityVector.plus(correctingVector).plus(centripetalCorrectionVector);
        driveVector.scaleToMagnitude(1);
        driveVector = driveVector.plus(turningVector);

        drive.setTargetVector(driveVector);
        velocityProfile.update();
    }

    public final ArrayList<GPose> trail = new ArrayList<>();

}
