package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

public class Intake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public enum State{
        START_INTAKE, INTAKE, STOP_INTAKE, STOP_INTAKE_THEN_REVERSE, REVERSE, IDLE
    }

    State state;

    public void setState(State newState){
        if(state == newState) return;

        switch (newState){
            case START_INTAKE:
                activeIntake.setState(ActiveIntake.State.IDLE);
                if(leftGripper.getState() == LeftGripper.State.CLOSED || leftGripper.getState() == LeftGripper.State.CLOSING)
                    leftGripper.setState(LeftGripper.State.OPENING);
                if(rightGripper.getState() == RightGripper.State.CLOSED || rightGripper.getState() == RightGripper.State.CLOSING)
                    rightGripper.setState(RightGripper.State.OPENING);
                dropDown.setState(DropDown.State.GOING_INTAKE);
                break;
            case INTAKE:
                activeIntake.setState(ActiveIntake.State.RUNNING);
                break;
            case STOP_INTAKE:
            case STOP_INTAKE_THEN_REVERSE:
                dropDown.setState(DropDown.State.GOING_UP);
                if(leftGripper.getState() == LeftGripper.State.OPEN || leftGripper.getState() == LeftGripper.State.OPENING)
                    leftGripper.setState(LeftGripper.State.CLOSING);
                if(rightGripper.getState() == RightGripper.State.OPEN || rightGripper.getState() == RightGripper.State.OPENING)
                    rightGripper.setState(RightGripper.State.CLOSING);
                break;
            case REVERSE:
                activeIntake.setState(ActiveIntake.State.REVERSE);
                break;
            case IDLE:
                activeIntake.setState(ActiveIntake.State.IDLE);
                break;
        }

        this.state = newState;
    }

    public State getState(){
        return state;
    }

    public final ActiveIntake activeIntake;
    public final LeftGripper leftGripper;
    public final RightGripper rightGripper;
    public final DropDown dropDown;

    public Intake(ActiveIntake activeIntake, LeftGripper leftGripper, RightGripper rightGripper, DropDown dropDown, State initialState){
        this.activeIntake = activeIntake;
        this.leftGripper = leftGripper;
        this.rightGripper = rightGripper;
        this.dropDown = dropDown;
        this.state = initialState;
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateState();
        updateHardware();
    }

    @Override
    public void atStart(){
        if(!ENABLED) return;

        activeIntake.atStart();
        leftGripper.atStart();
        rightGripper.atStart();
        dropDown.atStart();
    }

    @Override
    public void initUpdate(){
        if(!ENABLED) return;

        activeIntake.initUpdate();
        leftGripper.initUpdate();
        rightGripper.initUpdate();
        dropDown.initUpdate();
    }

    @Override
    public void updateState() {
        switch (state){
            case START_INTAKE:
                if(leftGripper.getState() == LeftGripper.State.OPEN && rightGripper.getState() == RightGripper.State.OPEN)
                    setState(State.INTAKE);
                break;
            case STOP_INTAKE:
                if(leftGripper.getState() == LeftGripper.State.CLOSED && rightGripper.getState() == RightGripper.State.CLOSED)
                    setState(State.IDLE);
                break;
            case STOP_INTAKE_THEN_REVERSE:
                if(leftGripper.getState() == LeftGripper.State.CLOSED && rightGripper.getState() == RightGripper.State.CLOSED)
                    setState(State.REVERSE);
                break;
        }
    }

    @Override
    public void updateHardware() {
        activeIntake.update();
        leftGripper.update();
        rightGripper.update();
        dropDown.update();
    }
}
