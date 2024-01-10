package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

public class Outtake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public enum State{
        GOING_UP, UP,
        GOING_DOWN, ARM_GOING_BACK_LIFT_GOING_DOWN, LIFT_DOWN, DOWN,
        LIFT_CHANGING_SCORING_POSITION,
        GOING_PURPLE, LIFT_BACK_DOWN_PURPLE ,PURPLE
    }

    State state;

    public void setState(State newState){
        if(state == newState) return;

        state = newState;

        switch (newState){
            case LIFT_CHANGING_SCORING_POSITION:
                lift.setState(Lift.State.GOING_UP);
                break;
            case GOING_UP:
                lift.setState(Lift.State.GOING_UP);
                arm.setState(OuttakeArm.State.GOING_OUTTAKE);
                pitch.setState(Pitch.State.GOING_OUTTAKE);
                break;
            case GOING_DOWN:
                setState(State.ARM_GOING_BACK_LIFT_GOING_DOWN);
                break;
            case ARM_GOING_BACK_LIFT_GOING_DOWN:
                lift.setState(Lift.State.GOING_PASSTHROUGH);
                arm.setState(OuttakeArm.State.GOING_INTAKE);
                pitch.setState(Pitch.State.GOING_INTAKE);
                break;
            case LIFT_DOWN:
                lift.setState(Lift.State.GOING_DOWN);
                break;
            case GOING_PURPLE:
                arm.setState(OuttakeArm.State.GOING_OUTTAKE);
                pitch.setState(Pitch.State.GOING_PURPLE);
                lift.setState(Lift.State.GOING_PASSTHROUGH);
                break;
            case LIFT_BACK_DOWN_PURPLE:
                lift.setState(Lift.State.GOING_DOWN);
                break;
        }
    }

    public State getState(){
        return state;
    }

    public final Lift lift;
    public final OuttakeArm arm;
    public final Pitch pitch;

    public Outtake(Lift lift, OuttakeArm arm, Pitch pitch, State initialState){
        this.lift = lift;
        this.arm = arm;
        this.pitch = pitch;
        this.state = initialState;
    }

    @Override
    public void atStart(){
        lift.atStart();
        arm.atStart();
        pitch.atStart();
    }

    @Override
    public void initUpdate(){
        lift.initUpdate();
        arm.initUpdate();
        pitch.initUpdate();
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        switch (state){
            case LIFT_CHANGING_SCORING_POSITION:
            case GOING_UP:
                if(lift.getState() == Lift.State.UP)
                    setState(State.UP);
                break;
            case ARM_GOING_BACK_LIFT_GOING_DOWN:
                if(arm.getState() == OuttakeArm.State.INTAKE)
                    setState(State.LIFT_DOWN);
                break;
            case LIFT_DOWN:
                if(lift.getState() == Lift.State.DOWN)
                    setState(State.DOWN);
                break;
            case GOING_PURPLE:
                if(arm.getState() == OuttakeArm.State.OUTTAKE && pitch.getState() == Pitch.State.PURPLE)
                    setState(State.LIFT_BACK_DOWN_PURPLE);
                break;
            case LIFT_BACK_DOWN_PURPLE:
                if(lift.getState() == Lift.State.DOWN)
                    setState(State.PURPLE);
                break;
        }
    }

    @Override
    public void updateHardware() {
        lift.update();
        arm.update();
        pitch.update();
    }
}
