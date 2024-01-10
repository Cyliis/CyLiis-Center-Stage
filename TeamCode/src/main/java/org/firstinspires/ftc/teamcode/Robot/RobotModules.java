package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.*;
import org.firstinspires.ftc.teamcode.Modules.LeftPixelColorSensor;

public class RobotModules implements IRobotModule {

    public ActiveIntake activeIntake;
    public DropDown dropDown;
    public Intake intake;
    public LeftGripper leftGripper;
    public Lift lift;
    public Outtake outtake;
    public OuttakeArm outtakeArm;
    public Pitch pitch;
    public RightGripper rightGripper;
    public Climber climber;
    public Plane plane;

    public LeftPixelColorSensor leftSensor;
    public RightPixelColorSensor rightSensor;

    public RobotModules(Hardware hardware){
        activeIntake = new ActiveIntake(hardware, ActiveIntake.State.IDLE);
        leftGripper = new LeftGripper(hardware, LeftGripper.State.CLOSING);
        lift = new Lift(hardware, Lift.State.GOING_DOWN);
        outtakeArm = new OuttakeArm(hardware, OuttakeArm.State.INTAKE);
        pitch = new Pitch(hardware, Pitch.State.INTAKE);
        rightGripper = new RightGripper(hardware, RightGripper.State.CLOSING);
        dropDown = new DropDown(hardware, DropDown.State.UP);

        climber = new Climber(hardware, Climber.State.IDLE);
        plane = new Plane(hardware, Plane.State.CLOSED);

        leftSensor = hardware.leftSensor;
        rightSensor = hardware.rightSensor;

        intake = new Intake(activeIntake, leftGripper, rightGripper, dropDown, Intake.State.IDLE);
        outtake = new Outtake(lift, outtakeArm, pitch, Outtake.State.DOWN);
    }

    public void telemetry(Telemetry telemetry){
        telemetry.addData("Lift level", Lift.level);
        telemetry.addData("Dropdown level", DropDown.index);
//        telemetry.addData("Lift target position", lift.profile.finalPosition);
//        telemetry.addData("Lift current position", lift.encoder.getCurrentPosition());
//        if(lift.encoder!=null) telemetry.addData("Lift current position", outtake.lift.encoder.getCurrentPosition());
//        telemetry.addData("Lift current state", lift.getState());
//        telemetry.addData("Adaptable passthrough position", lift.passthroughPosition(Lift.outtakeArmPosition));
//        if(lift.encoder!=null) telemetry.addData("Lift ground", Lift.groundPos);
        telemetry.addData("Intake state", intake.getState());
        telemetry.addData("Outtake state", outtake.getState());
//        telemetry.addData("Active intake power", activeIntake.getState().power);
        telemetry.addData("Plane state", plane.getState());
//        telemetry.addData("Outtake arm state", outtakeArm.getState());
//        if(outtakeArm.leftServo != null) telemetry.addData("Outtake arm position", outtakeArm.leftServo.cachedPosition);
//        telemetry.addData("Outtake arm position lift", Math.cos((Lift.outtakeArmPosition - OuttakeArm.State.VERTICAL.position) * Math.toRadians(270)) * Lift.armLength + Lift.armFloor);
//        telemetry.addData("Active intake state", activeIntake.getState());
//        telemetry.addData("Left gripper state", leftGripper.getState());
//        telemetry.addData("Right gripper state", rightGripper.getState());
//        telemetry.addData("Dropdown state", dropDown.getState());
//        telemetry.addData("Left gripper state", leftGripper.getState());
//        telemetry.addData("Right gripper state", rightGripper.getState());
    }
    @Override
    public void initUpdate() {
        if(Intake.ENABLED) intake.initUpdate();
        if(Outtake.ENABLED) outtake.initUpdate();
        if(Climber.ENABLED) climber.initUpdate();
        if(Plane.ENABLED) plane.initUpdate();
    }

    @Override
    public void atStart() {
        if(Intake.ENABLED) intake.atStart();
        if(Outtake.ENABLED) outtake.atStart();
        if(Climber.ENABLED) climber.atStart();
        if(Plane.ENABLED) plane.atStart();
    }

    @Override
    public void update() {
        if(Intake.ENABLED) intake.update();
        if(Outtake.ENABLED) outtake.update();
        if(Climber.ENABLED) climber.update();
        if(Plane.ENABLED) plane.update();
    }

    @Override
    public void emergencyStop() {
        if(Intake.ENABLED) intake.emergencyStop();
        if(Outtake.ENABLED) outtake.emergencyStop();
        if(Climber.ENABLED) climber.emergencyStop();
        if(Plane.ENABLED) plane.emergencyStop();
    }
}
