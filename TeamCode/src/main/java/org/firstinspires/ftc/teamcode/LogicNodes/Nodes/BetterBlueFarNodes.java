package org.firstinspires.ftc.teamcode.LogicNodes.Nodes;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LogicNodes.LogicNode;
import org.firstinspires.ftc.teamcode.Modules.DropDown;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.LeftGripper;
import org.firstinspires.ftc.teamcode.Modules.RightGripper;
import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;

public class BetterBlueFarNodes {
    public ElapsedTime autoTimer = new ElapsedTime();
    public double cycleTime = 8.5, returnTime = 6;

    public double returnTimeLeft = 4.5, returnTimeMiddle = 5.5, returnTimeRight = 5.5;

    public LogicNode currentNode = new LogicNode("Nothing");

    private int detectionCase;

    public final LogicNode start = new LogicNode("Start");

    private final LogicNode placePurple = new LogicNode("Placing purple");

    private final LogicNode backUpFromPurple = new LogicNode("Backing up from pixel");

    private final LogicNode alignForIntake = new LogicNode("Aligning to intake");

    private final LogicNode turnForIntake = new LogicNode("Turning for intake");

    private final LogicNode goToIntake = new LogicNode("Going to intake position");

    private final LogicNode alignAgainToCrossBack = new LogicNode("Aligning again to cross back");

    private final LogicNode intake = new LogicNode("Intaking");

    private final LogicNode reverseToRetry = new LogicNode("Reversing then retrying");

    private final LogicNode retryIntake = new LogicNode("Retrying to intake");

    private final LogicNode reverseToLeave = new LogicNode("Reversing intake to leave");

    private final LogicNode crossField = new LogicNode("Crossing field");

    private final LogicNode alignAgainToCross = new LogicNode("Aligning again to cross");

    private final LogicNode goToScore = new LogicNode("Go to score");

    private final LogicNode waitToScore = new LogicNode("Waiting to score");

    private final LogicNode alignToCrossBack = new LogicNode("Aligning to cross back");

    private final LogicNode park = new LogicNode("Parking");

    private final LogicNode end = new LogicNode("End");

    private final ElapsedTime timer = new ElapsedTime();

    private int cycles = 0;

    public Pose offset = new Pose();

    public static double xIncrement = 1;
    public static double xDriftIncrement = 0;
    public static double yDriftIncrement = 0;

    private final Pose driftAfterLeft = new Pose(0,-1.5);
    private final Pose driftAfterMiddle = new Pose(0, -1);
    private final Pose driftAfterRight = new Pose(0, -1.5);
    private Pose driftAfterPurple = new Pose();

    public static double timeToScore = 0.85, timeToIntake = 1.3, timeToReverse = 0.85;
    public static double movementTimeOut = 0.15;

    private int intakeTries = 0;

    private final Pose purpleLeftPose = new Pose(27, 3, PI/2.0);
    private final Pose purpleMiddlePose = new Pose(30.5, 2, 0.88);
    private final Pose purpleRightPose = new Pose(20, -2.5, 0);

    private Pose purplePose = new Pose();

    private final Pose backUpPoseLeft = new Pose(27, 0, PI/2.0);
    private final Pose backUpPoseMiddle = new Pose(25,-2,0.88);
    private final Pose backUpPoseRight = new Pose(10, 5, 0);

    private Pose backUpPose = new Pose();

    private final Pose lineUpForIntakePoseLeft = new Pose(50,-6,PI/2.0);
    private final Pose lineUpForIntakePoseMiddle = new Pose(49, -10, PI/2.0);
    private final Pose lineUpForIntakePoseRight = new Pose(51.5, 7, PI/2.0);

    private Pose lineUpForIntakePose = new Pose();


//    private Pose intakePoseLeft = new Pose(50,-20.5,1.56);
//    private Pose intakePoseMiddle = new Pose(49,-20.5,1.56);
//    private Pose intakePoseRight = new Pose(49,-20.5,1.56);

    private Pose intakePose = new Pose(51,-20,1.56);

    private final Pose crossFieldPose = new Pose(54,69,1.55);

    private final Pose scoringPoseLeftYellow = new Pose(21,89,1.55);
    private final Pose scoringPoseMiddleYellow = new Pose(25,88.8,1.55);
    private final Pose scoringPoseRightYellow = new Pose(30,88.8,1.55);

    private Pose scoringPoseYellow = new Pose();

    private final Pose scoringPoseLeft = new Pose(26,89,1.55);
    private final Pose scoringPoseMiddle = new Pose(30,89,1.55);
    private final Pose scoringPoseRight = new Pose(25,88.8,1.55);

    private Pose scoringPose = new Pose();

    private final Pose crossBackPose = new Pose(52.5,69,1.57);

    private final Pose parkPose = new Pose(25,85,PI/2.0);

    public BetterBlueFarNodes(MecanumDrive drive, RobotModules robot, int detectionCase){
        drive.setRunMode(MecanumDrive.RunMode.PID);
        CoolIMU.imuOffset = 0;
        switch (detectionCase){
            case 1:
                purplePose = purpleLeftPose;
                backUpPose = backUpPoseLeft;
                lineUpForIntakePose = lineUpForIntakePoseLeft;
                driftAfterPurple = driftAfterLeft;
                scoringPose = scoringPoseLeft;
                scoringPoseYellow = scoringPoseLeftYellow;
                returnTime = returnTimeLeft;
                break;
            case 2:
                purplePose = purpleMiddlePose;
                backUpPose = backUpPoseMiddle;
                lineUpForIntakePose = lineUpForIntakePoseMiddle;
                driftAfterPurple = driftAfterMiddle;
                scoringPose = scoringPoseMiddle;
                scoringPoseYellow = scoringPoseMiddleYellow;
                returnTime = returnTimeMiddle;
                break;
            case 3:
                purplePose = purpleRightPose;
                backUpPose = backUpPoseRight;
                lineUpForIntakePose = lineUpForIntakePoseRight;
                driftAfterPurple = driftAfterRight;
                scoringPose = scoringPoseRight;
                scoringPoseYellow = scoringPoseRightYellow;
                returnTime = returnTimeRight;
                break;
        }
        this.detectionCase = detectionCase;
        initNodes(drive, robot);
    }

    private void initNodes(MecanumDrive drive, RobotModules robot){
        start.addCondition(()->detectionCase == 1, ()->{
            autoTimer.reset();
            drive.setTargetPose(purplePose);
            timer.reset();
        }, placePurple);
        start.addCondition(()->detectionCase == 2, ()->{
            autoTimer.reset();
            drive.setTargetPose(purplePose);
            timer.reset();
        }, placePurple);
        start.addCondition(()->detectionCase == 3, ()->{
            autoTimer.reset();
            drive.setTargetPose(purplePose);
            timer.reset();
        }, placePurple);

        placePurple.addPositionCondition(drive, 1, backUpPose, backUpFromPurple);

        backUpFromPurple.addPositionCondition(drive, 2, lineUpForIntakePose, alignForIntake);

        alignForIntake.addCondition(()->drive.reachedTarget(2),
                ()->{
                    drive.setTargetPose(new Pose(drive.getTargetPose().getX(), drive.getTargetPose().getY(), PI/2.0));
                    offset = offset.plus(driftAfterPurple);
                }, turnForIntake);

        turnForIntake.addCondition(()->drive.reachedHeading(0.1), ()->{
            robot.intake.setState(Intake.State.START_INTAKE);
            if(cycles == 0) DropDown.index = 4;
            drive.setTargetPose(intakePose.plus(offset));
            timer.reset();
        }, goToIntake);

        goToIntake.addCondition(()->drive.reachedTarget(3), ()->{
            timer.reset();
            intakeTries = 1;
        }, intake);
        goToIntake.addCondition(()->timer.seconds() >= movementTimeOut && drive.stopped() && !drive.reachedTarget(3), ()->{
            offset = offset.plus(new Pose(xIncrement, 0));
            drive.setTargetPose(drive.getLocalizer().getPoseEstimate().plus(offset));
        }, alignAgainToCrossBack);

        alignAgainToCrossBack.addPositionCondition(drive, 2, crossFieldPose.plus(offset), goToIntake);

        intake.addCondition(()->robot.leftSensor.isPixelUpdate() && robot.rightSensor.isPixelUpdate(), ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            timer.reset();
        }, reverseToLeave);
        intake.addCondition(()-> timer.seconds() >= timeToIntake && (!robot.leftSensor.isPixelUpdate() || !robot.rightSensor.isPixelUpdate()), ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            timer.reset();
        }, reverseToRetry);
        intake.addCondition(()->30 - autoTimer.seconds() <= returnTime, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            timer.reset();
        }, reverseToLeave);

        reverseToRetry.addCondition(()->timer.seconds() >= timeToReverse, ()->{
            robot.intake.setState(Intake.State.START_INTAKE);
            DropDown.index = Math.max(DropDown.index - 1, 0);
            timer.reset();
        }, retryIntake);

        retryIntake.addCondition(()->((robot.leftSensor.isPixelUpdate() && robot.rightSensor.isPixelUpdate()) ||
                (robot.leftSensor.isPixelUpdate() && intakeTries == 3) || (robot.rightSensor.isPixelUpdate() && intakeTries == 3)), ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            timer.reset();
        }, reverseToLeave);
        retryIntake.addCondition(()-> timer.seconds() >= timeToIntake && (!robot.leftSensor.isPixelUpdate() || !robot.rightSensor.isPixelUpdate()), ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            intakeTries++;
            timer.reset();
        }, reverseToRetry);
        retryIntake.addCondition(()->30 - autoTimer.seconds() <= returnTime, ()->{
            robot.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
            timer.reset();
        }, reverseToLeave);

        reverseToLeave.addCondition(()->timer.seconds() >= timeToReverse, ()->{
            drive.setTargetPose(crossFieldPose.plus(offset));
            timer.reset();
        }, crossField);

        crossField.addCondition(()->timer.seconds() >= movementTimeOut && drive.stopped() && !drive.reachedTarget(4), ()->{
            offset = offset.plus(new Pose(xIncrement, 0));
            drive.setTargetPose(drive.getLocalizer().getPoseEstimate().plus(offset));
        }, alignAgainToCross);

        alignAgainToCross.addPositionCondition(drive,2, crossFieldPose.plus(offset), crossField);

        crossField.addCondition(()->drive.reachedTarget(10), ()->{
            robot.intake.setState(Intake.State.IDLE);
            drive.setTargetPose(cycles == 0?scoringPoseYellow.plus(offset):scoringPose.plus(offset));
            if(cycles == 0) Lift.level = 0;
            else Lift.level = 3;
            if(robot.leftSensor.isPixelUpdate()) DropDown.index = Math.max(DropDown.index - 1, 0);
            if(robot.rightSensor.isPixelUpdate()) DropDown.index = Math.max(DropDown.index - 1, 0);

            robot.outtake.setState(Outtake.State.GOING_UP);
            timer.reset();
        }, goToScore);

        goToScore.addCondition(()->drive.reachedTarget(0.3), ()->{
            robot.leftGripper.setState(LeftGripper.State.OPENING);
            robot.rightGripper.setState(RightGripper.State.OPENING);
            timer.reset();
        }, waitToScore);

        goToScore.addCondition(()->timer.seconds() >= movementTimeOut && drive.stopped() && !drive.reachedTarget(0.3), ()->{
            robot.leftGripper.setState(LeftGripper.State.OPENING);
            robot.rightGripper.setState(RightGripper.State.OPENING);
            timer.reset();
        },waitToScore);

        waitToScore.addCondition(()->timer.seconds() >= timeToScore && 30 - autoTimer.seconds() < cycleTime, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            drive.setTargetPose(parkPose.plus(offset));
        }, park);

        waitToScore.addCondition(()->timer.seconds() >= timeToScore && 30-autoTimer.seconds() >= cycleTime, ()->{
            robot.outtake.setState(Outtake.State.GOING_DOWN);
            drive.setTargetPose(crossBackPose.plus(offset));
            cycles++;
        }, alignToCrossBack);

        alignToCrossBack.addCondition(()->drive.reachedTarget(6), ()->{
            offset = offset.plus(new Pose(xDriftIncrement, yDriftIncrement));
            drive.setTargetPose(intakePose.plus(offset));
            robot.intake.setState(Intake.State.START_INTAKE);
            timer.reset();
        }, goToIntake);

        park.addPositionCondition(drive, 2, parkPose.plus(offset), end);

    }
}
