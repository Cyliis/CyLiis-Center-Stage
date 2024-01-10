package org.firstinspires.ftc.teamcode.Robot.GamepadControllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.*;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

public class BuruGamepadControl implements IRobotModule {

    private final RobotModules robotModules;
    private final StickyGamepad stickyGamepad1, stickyGamepad2;
    private final Gamepad gamepad1, gamepad2;

    public BuruGamepadControl(RobotModules robotModules, Gamepad gamepad1, Gamepad gamepad2){
        this.robotModules = robotModules;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
        this.stickyGamepad2 = new StickyGamepad(gamepad2);
    }

    private void intakeControl(){
        if(robotModules.outtake.getState()!= Outtake.State.DOWN) return;
        if(gamepad1.b && (robotModules.intake.getState() != Intake.State.INTAKE && robotModules.intake.getState() != Intake.State.START_INTAKE)) {
            robotModules.intake.setState(Intake.State.START_INTAKE);
        }
        if(gamepad1.b && (robotModules.intake.getState() == Intake.State.INTAKE || robotModules.intake.getState() == Intake.State.START_INTAKE)){
            robotModules.intake.setState(Intake.State.STOP_INTAKE);
        }
        if(gamepad1.y && (robotModules.intake.getState() == Intake.State.INTAKE || robotModules.intake.getState() == Intake.State.START_INTAKE)){
            robotModules.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
        }
        if(gamepad1.y && (robotModules.intake.getState() != Intake.State.INTAKE && robotModules.intake.getState() != Intake.State.START_INTAKE)){
            robotModules.intake.setState(Intake.State.REVERSE);
        }
        if(gamepad1.y && robotModules.intake.getState() == Intake.State.REVERSE){
            robotModules.intake.setState(Intake.State.IDLE);
        }
    }

    public void outtakeControl(){
        if(robotModules.intake.getState() != Intake.State.IDLE) return;
        if(stickyGamepad1.dpad_right){
            if(robotModules.outtake.getState() == Outtake.State.UP) robotModules.outtake.setState(Outtake.State.GOING_DOWN);
            else if(robotModules.outtake.getState() == Outtake.State.DOWN) robotModules.outtake.setState(Outtake.State.GOING_UP);
        }
        if(stickyGamepad1.dpad_up){
            Lift.level = Math.min(8, Lift.level + 1);
            if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.LIFT_CHANGING_SCORING_POSITION) robotModules.outtake.setState(Outtake.State.LIFT_CHANGING_SCORING_POSITION);
        }
        if(stickyGamepad1.dpad_down){
            Lift.level = Math.max(0, Lift.level - 1);
            if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.LIFT_CHANGING_SCORING_POSITION) robotModules.outtake.setState(Outtake.State.LIFT_CHANGING_SCORING_POSITION);
        }
    }

    public void grippersControl(){
        if(robotModules.outtake.getState() != Outtake.State.UP) return;
        if(robotModules.leftGripper.getState() == LeftGripper.State.CLOSED && stickyGamepad1.x) robotModules.leftGripper.setState(LeftGripper.State.OPENING);
        if(robotModules.rightGripper.getState() == RightGripper.State.CLOSED && stickyGamepad1.a) robotModules.rightGripper.setState(RightGripper.State.OPENING);
    }

    @Override
    public void update() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        intakeControl();
        outtakeControl();
        grippersControl();
    }
}
