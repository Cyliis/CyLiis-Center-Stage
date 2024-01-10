package org.firstinspires.ftc.teamcode.LogicNodes.Nodes;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.LogicNode;
import org.firstinspires.ftc.teamcode.Modules.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.LeftGripper;
import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.RightGripper;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTangentHeadingTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class SplineRedFarNodes {
    RobotModules robot;
    Follower follower;
    MecanumDrive drive;
    
    public final LogicNode currentNode;
    
    private static class Trajectories {

        public static Trajectory purplePixelTrajectory;
        public static Trajectory firstIntakeTrajectory;
        public static Trajectory yellowPixelTrajectory;
        public static Trajectory firstCycleIntakeTrajectory;
        public static Trajectory firstCycleScoreTrajectory;
        public static Trajectory secondCycleIntakeTrajectory;
        public static Trajectory secondCycleScoreTrajectory;
        public static Trajectory park;

        public static CubicBezierTangentHeadingTrajectorySegment purpleSegmentLeft;
        public static CubicBezierTangentHeadingTrajectorySegment firstIntakeSegmentLeft;
        public static CubicBezierTangentHeadingTrajectorySegment yellowSegmentLeft;
        public static CubicBezierTangentHeadingTrajectorySegment firstCycleIntakeSegmentLeft;
        public static CubicBezierTangentHeadingTrajectorySegment firstCycleScoreSegmentLeft;
        public static CubicBezierTangentHeadingTrajectorySegment secondCycleIntakeSegmentLeft;
        public static CubicBezierTangentHeadingTrajectorySegment secondCycleScoreSegmentLeft;
        public static CubicBezierTangentHeadingTrajectorySegment parkLeft;

        public static CubicBezierTangentHeadingTrajectorySegment purpleSegmentMiddle;
        public static CubicBezierTangentHeadingTrajectorySegment firstIntakeSegmentMiddle;
        public static CubicBezierTangentHeadingTrajectorySegment yellowSegmentMiddle;
        public static CubicBezierTangentHeadingTrajectorySegment firstCycleIntakeSegmentMiddle;
        public static CubicBezierTangentHeadingTrajectorySegment firstCycleScoreSegmentMiddle;
        public static CubicBezierTangentHeadingTrajectorySegment secondCycleIntakeSegmentMiddle;
        public static CubicBezierTangentHeadingTrajectorySegment secondCycleScoreSegmentMiddle;
        public static CubicBezierTangentHeadingTrajectorySegment parkMiddle;

        public static CubicBezierTrajectorySegment purpleSegmentRight =
            new CubicBezierTrajectorySegment(
                    new Pose(0,0,0),
                    new Pose(19.6, 4, -PI/4.0),
                    new Pose(25.5, 8.5, -PI/2.0),
                    new Pose(27.5, -6,-PI/2.0)
            );
        public static CubicBezierTrajectorySegment firstIntakeSegmentRight =
            new CubicBezierTrajectorySegment(
                    new Pose(27.5, -6, -PI/2.0),
                    new Pose(26.6, 12.4, -PI/2.0),
                    new Pose(50, -10.3, -PI/2.0),
                    new Pose(50, 20.5, -PI/2.0)
            );
        public static CubicBezierTrajectorySegment yellowSegmentRight =
            new CubicBezierTrajectorySegment(
                    new Pose(50, 20.5, -PI/2.0),
                    new Pose(57.3, -115, -PI/2.0),
                    new Pose(21.7, -44.8, -PI/2.0),
                    new Pose(19, -90, -PI/2.0)
            );
        public static CubicBezierTrajectorySegment firstCycleIntakeSegmentRight =
            new CubicBezierTrajectorySegment(
                    new Pose(20, -90, -PI/2.0),
                    new Pose(17.4, -51, -PI/2.0),
                    new Pose(55, -115, -PI/2.0),
                    new Pose(50.5, 21, -PI/2.0)
            );
        public static CubicBezierTrajectorySegment firstCycleScoreSegmentRight =
            new CubicBezierTrajectorySegment(
                    new Pose(51.5, 21, -PI/2.0),
                    new Pose(57.4, -115, -PI/2.0),
                    new Pose(28.3, -50, -PI/2.0),
                    new Pose(30.5, -89.5, -PI/2.0)
            );
        public static CubicBezierTrajectorySegment secondCycleIntakeSegmentRight =
                new CubicBezierTrajectorySegment(
                        new Pose(30.5, -88.5, -PI/2.0),
                        new Pose(30, -50, -PI/2.0),
                        new Pose(59, -115, -PI/2.0),
                        new Pose(50.5, 21, -PI/2.0)
                );
        public static CubicBezierTrajectorySegment secondCycleScoreSegmentRight =
                new CubicBezierTrajectorySegment(
                        new Pose(52.5, 21, -PI/2.0),
                        new Pose(56.5, -115, -PI/2.0),
                        new Pose(28, -55.7, -PI/2.0),
                        new Pose(30.5, -90, -PI/2.0)
                );
        public static CubicBezierTangentHeadingTrajectorySegment parkRight = new CubicBezierTangentHeadingTrajectorySegment(
                new Pose(30.5, -90.5, PI/2.0),
                new Pose(30.5, -90.5),
                new Pose(30.5, -86),
                new Pose(30.5, -86, PI/2.0),
                -PI
        );

    }
    private static class Nodes {
        public static final LogicNode start = new LogicNode("Start");
        public static final LogicNode placePurplePixel = new LogicNode("Placing purple pixel");
        public static final LogicNode goToIntakeFirstPixel = new LogicNode("Going to intake first pixel");
        public static final LogicNode scoreYellowPixel = new LogicNode("Scoring yellow pixel");
        public static final LogicNode goToIntake1 = new LogicNode("Going to intake 1");
        public static final LogicNode goToIntake2 = new LogicNode("Going to intake 2");
        public static final LogicNode intake = new LogicNode("Intaking");
        public static final LogicNode reverseToRetry = new LogicNode("Reversing then retrying");
        public static final LogicNode retryIntake = new LogicNode("Retrying to intake");
        public static final LogicNode reverseToLeave = new LogicNode("Reversing intake to leave");
        public static final LogicNode goToScore1 = new LogicNode("Going to score 1");
        public static final LogicNode goToScore2 = new LogicNode("Going to score 2");
        public static final LogicNode waitToScore = new LogicNode("Waiting to score");
        public static final LogicNode park = new LogicNode("Parking");
        public static final LogicNode end = new LogicNode("End");
    }

    public int detectionCase = 2;
    private int cycle = 0, tries = 0;
    public static int maxTries = 3;
    public static double intakeTime = 1.3, reverseTime = 0.85, dropTime = 0.8;
    public static double cycleTimeLeft = 7.5, cycleTimeMiddle = 7.75, cycleTimeRight = 8;
    private double cycleTime;
    public static double returnTimeLeft = 4.5, returnTimeMiddle = 4.75, returnTimeRight = 5;
    private double returnTime;
    public static double movementTimeOut = 0.3;

    public static double fieldYThresh = -50;

    private boolean liftUp = false, intakeStarted = false;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();

    public SplineRedFarNodes(RobotModules robot, Follower follower, MecanumDrive drive, int detectionCase){
        this.robot = robot;
        this.follower = follower;
        this.drive = drive;
        this.detectionCase = detectionCase;

        initTrajectories();
        initValues();
        initNodes();

        this.currentNode = Nodes.start;
    }
    
    void initTrajectories(){
        switch (detectionCase){
            case 1:
                Trajectories.purplePixelTrajectory = new TrajectoryBuilder(Trajectories.purpleSegmentLeft).build();
                Trajectories.firstIntakeTrajectory = new TrajectoryBuilder(Trajectories.firstIntakeSegmentLeft).build();
                Trajectories.yellowPixelTrajectory = new TrajectoryBuilder(Trajectories.yellowSegmentLeft).build();
                Trajectories.firstCycleIntakeTrajectory = new TrajectoryBuilder(Trajectories.firstCycleIntakeSegmentLeft).build();
                Trajectories.firstCycleScoreTrajectory = new TrajectoryBuilder(Trajectories.firstCycleScoreSegmentLeft).build();
                Trajectories.secondCycleIntakeTrajectory = new TrajectoryBuilder(Trajectories.secondCycleIntakeSegmentLeft).build();
                Trajectories.secondCycleScoreTrajectory = new TrajectoryBuilder(Trajectories.secondCycleScoreSegmentLeft).build();
                Trajectories.park = new TrajectoryBuilder(Trajectories.parkLeft).build();
                break;
            case 2:
                Trajectories.purplePixelTrajectory = new TrajectoryBuilder(Trajectories.purpleSegmentMiddle).build();
                Trajectories.firstIntakeTrajectory = new TrajectoryBuilder(Trajectories.firstIntakeSegmentMiddle).build();
                Trajectories.yellowPixelTrajectory = new TrajectoryBuilder(Trajectories.yellowSegmentMiddle).build();
                Trajectories.firstCycleIntakeTrajectory = new TrajectoryBuilder(Trajectories.firstCycleIntakeSegmentMiddle).build();
                Trajectories.firstCycleScoreTrajectory = new TrajectoryBuilder(Trajectories.firstCycleScoreSegmentMiddle).build();
                Trajectories.secondCycleIntakeTrajectory = new TrajectoryBuilder(Trajectories.secondCycleIntakeSegmentMiddle).build();
                Trajectories.secondCycleScoreTrajectory = new TrajectoryBuilder(Trajectories.secondCycleScoreSegmentMiddle).build();
                Trajectories.park = new TrajectoryBuilder(Trajectories.parkMiddle).build();
                break;
            case 3:
                Trajectories.purplePixelTrajectory = new TrajectoryBuilder(Trajectories.purpleSegmentRight).build();
                Trajectories.firstIntakeTrajectory = new TrajectoryBuilder(Trajectories.firstIntakeSegmentRight).build();
                Trajectories.yellowPixelTrajectory = new TrajectoryBuilder(Trajectories.yellowSegmentRight).build();
                Trajectories.firstCycleIntakeTrajectory = new TrajectoryBuilder(Trajectories.firstCycleIntakeSegmentRight).build();
                Trajectories.firstCycleScoreTrajectory = new TrajectoryBuilder(Trajectories.firstCycleScoreSegmentRight).build();
                Trajectories.secondCycleIntakeTrajectory = new TrajectoryBuilder(Trajectories.secondCycleIntakeSegmentRight).build();
                Trajectories.secondCycleScoreTrajectory = new TrajectoryBuilder(Trajectories.secondCycleScoreSegmentRight).build();
                Trajectories.park = new TrajectoryBuilder(Trajectories.parkRight).build();
                break;
        }
    }

    void initValues(){
        switch (detectionCase){
            case 1:
                cycleTime = cycleTimeLeft;
                returnTime = returnTimeLeft;
                break;
            case 2:
                cycleTime = cycleTimeMiddle;
                returnTime = returnTimeMiddle;
                break;
            case 3:
                cycleTime = cycleTimeRight;
                returnTime = returnTimeRight;
                break;
        }
    }

    void initNodes(){
        Nodes.start.addCondition(()->true, ()->{
            follower.setTrajectory(Trajectories.purplePixelTrajectory);
            cycle = 0;
            tries = 0;
            timer.startTime();
            autoTimer.startTime();
            autoTimer.reset();
            DropDown.index = 4;
        }, Nodes.placePurplePixel);

        Nodes.placePurplePixel.addCondition(()->drive.reachedTarget(1), ()->{
            follower.setTrajectory(Trajectories.firstIntakeTrajectory);
            robot.intake.setState(Intake.State.START_INTAKE);
        }, Nodes.goToIntakeFirstPixel);

        Nodes.goToIntakeFirstPixel.addCondition(()->drive.reachedTarget(2), ()->timer.reset(), Nodes.intake);
        Nodes.goToIntake1.addCondition(()->drive.reachedTarget(2), ()->timer.reset(), Nodes.intake);
        Nodes.goToIntake2.addCondition(()->drive.reachedTarget(2), ()->timer.reset(), Nodes.intake);

        Nodes.intake.addCondition(()->robot.leftSensor.isPixelUpdate() && robot.rightSensor.isPixelUpdate(), ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            timer.reset();
        }, Nodes.reverseToLeave);
        Nodes.intake.addCondition(()-> timer.seconds() >= intakeTime && (!robot.leftSensor.isPixelUpdate() || !robot.rightSensor.isPixelUpdate()), ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            tries = 1;
            timer.reset();
        }, Nodes.reverseToRetry);
        Nodes.intake.addCondition(()->30 - autoTimer.seconds() <= returnTime, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            tries = 1;
            timer.reset();
        }, Nodes.reverseToLeave);

        Nodes.reverseToRetry.addCondition(()->timer.seconds() >= reverseTime, ()->{
            robot.intake.setState(Intake.State.START_INTAKE);
            DropDown.index = Math.max(DropDown.index - 1, 0);
            timer.reset();
        }, Nodes.retryIntake);

        Nodes.retryIntake.addCondition(()->((robot.leftSensor.isPixelUpdate() && robot.rightSensor.isPixelUpdate()) ||
                (robot.leftSensor.isPixelUpdate() && tries == maxTries) || (robot.rightSensor.isPixelUpdate() && tries == maxTries)), ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            timer.reset();
        }, Nodes.reverseToLeave);
        Nodes.retryIntake.addCondition(()-> timer.seconds() >= intakeTime && (!robot.leftSensor.isPixelUpdate() || !robot.rightSensor.isPixelUpdate()), ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            tries++;
            timer.reset();
        }, Nodes.reverseToRetry);
        Nodes.retryIntake.addCondition(()->30 - autoTimer.seconds() <= returnTime, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            timer.reset();
        }, Nodes.reverseToLeave);

        Nodes.reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle == 0, ()->{
            follower.setTrajectory(Trajectories.yellowPixelTrajectory);
            timer.reset();
            liftUp = false;
            intakeStarted = false;
        }, Nodes.scoreYellowPixel);
        Nodes.reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle == 1, ()->{
            follower.setTrajectory(Trajectories.firstCycleScoreTrajectory);
            timer.reset();
            liftUp = false;
            intakeStarted = false;
        }, Nodes.goToScore1);
        Nodes.reverseToLeave.addCondition(()->timer.seconds() >= reverseTime && cycle == 2, ()->{
            follower.setTrajectory(Trajectories.secondCycleScoreTrajectory);
            timer.reset();
            liftUp = false;
        }, Nodes.goToScore2);

        Nodes.scoreYellowPixel.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= fieldYThresh && !liftUp, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            if(detectionCase == 1) DropDown.index = 4;
            if(robot.rightSensor.isPixelUpdate()) DropDown.index = Math.max(DropDown.index - 1, 0);
            if(robot.leftSensor.isPixelUpdate()) DropDown.index = Math.max(DropDown.index - 1, 0);

            Lift.level = 0;
            robot.outtake.setState(Outtake.State.GOING_UP);
            liftUp = true;
        }, Nodes.scoreYellowPixel);

        Nodes.goToScore1.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= fieldYThresh && !liftUp, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            if(robot.rightSensor.isPixelUpdate()) DropDown.index = Math.max(DropDown.index - 1, 0);
            if(robot.leftSensor.isPixelUpdate()) DropDown.index = Math.max(DropDown.index - 1, 0);

            Lift.level = 3;
            robot.outtake.setState(Outtake.State.GOING_UP);
            liftUp = true;
        }, Nodes.goToScore1);

        Nodes.goToScore2.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() <= fieldYThresh && !liftUp, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE);
            if(robot.rightSensor.isPixelUpdate()) DropDown.index = Math.max(DropDown.index - 1, 0);
            if(robot.leftSensor.isPixelUpdate()) DropDown.index = Math.max(DropDown.index - 1, 0);

            Lift.level = 3;
            robot.outtake.setState(Outtake.State.GOING_UP);
            liftUp = true;
        }, Nodes.goToScore2);

        Nodes.scoreYellowPixel.addCondition(()->drive.reachedTarget(1.5) || (drive.stopped() && timer.seconds() >= movementTimeOut), ()->{
            robot.leftGripper.setState(LeftGripper.State.OPENING);
            robot.rightGripper.setState(RightGripper.State.OPENING);
            timer.reset();
        }, Nodes.waitToScore);

        Nodes.goToScore1.addCondition(()->drive.reachedTarget(1.5) || (drive.stopped() && timer.seconds() >= movementTimeOut), ()->{
            robot.leftGripper.setState(LeftGripper.State.OPENING);
            robot.rightGripper.setState(RightGripper.State.OPENING);
            timer.reset();
        }, Nodes.waitToScore);

        Nodes.goToScore2.addCondition(()->drive.reachedTarget(1.5) || (drive.stopped() && timer.seconds() >= movementTimeOut), ()->{
            robot.leftGripper.setState(LeftGripper.State.OPENING);
            robot.rightGripper.setState(RightGripper.State.OPENING);
            timer.reset();
        }, Nodes.waitToScore);

        Nodes.waitToScore.addCondition(()->timer.seconds() >= dropTime && 30.0 - autoTimer.seconds() < cycleTime, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            follower.setTrajectory(Trajectories.park);
            cycle++;
        },Nodes.park);
        Nodes.waitToScore.addCondition(()->timer.seconds() >= dropTime && cycle == 0, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            follower.setTrajectory(Trajectories.firstCycleIntakeTrajectory);
            cycle++;
        },Nodes.goToIntake1);
        Nodes.waitToScore.addCondition(()->timer.seconds() >= dropTime && cycle == 1, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            follower.setTrajectory(Trajectories.secondCycleIntakeTrajectory);
            cycle++;
        },Nodes.goToIntake2);
        Nodes.waitToScore.addCondition(()->timer.seconds() >= dropTime && cycle == 2, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            follower.setTrajectory(Trajectories.park);
            cycle++;
        },Nodes.park);

        Nodes.goToIntake1.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() >= fieldYThresh && !intakeStarted, ()->{
            robot.intake.setState(Intake.State.START_INTAKE);
            intakeStarted = true;
        }, Nodes.goToIntake1);

        Nodes.goToIntake2.addCondition(()->drive.getLocalizer().getPoseEstimate().getY() >= fieldYThresh && !intakeStarted, ()->{
            robot.intake.setState(Intake.State.START_INTAKE);
            intakeStarted = true;
        }, Nodes.goToIntake2);

    }
}