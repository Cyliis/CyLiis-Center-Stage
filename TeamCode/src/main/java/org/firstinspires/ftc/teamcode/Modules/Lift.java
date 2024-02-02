package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Math.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Wrappers.EncoderFunny;

@Config
public class Lift implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    private final CoolMotor leftMotor, rightMotor;
    public static boolean leftMotorReversed = false, rightMotorReversed = true;
    public final Encoder encoder;
    public static boolean encoderReversed = false;

    public static int groundPos = 0, firstLevel = 145, increment = 70, level = 0, positionThresh = 16, passthroughPosition = 100;

    public static double resetPower = -0.5, velocityThreshold = 0;

    public static PIDCoefficients pid = new PIDCoefficients(0.015,0.15,0.00045);
    public static double ff1 = 0.14, ff2 = 0.0002;

    public static double maxVelocity = 12000, acceleration = 10000, deceleration = 3500;
//    public static double maxVelocity = 20000, acceleration = 12000, deceleration = 12000;
    public AsymmetricMotionProfile profile = new AsymmetricMotionProfile(maxVelocity, acceleration, deceleration);

    public enum State{
        DOWN(groundPos), RESETTING(groundPos, DOWN), GOING_DOWN(groundPos, RESETTING), PASSTHROUGH(groundPos + passthroughPosition), GOING_PASSTHROUGH(groundPos + passthroughPosition, PASSTHROUGH),
        UP(groundPos + firstLevel + increment * level), GOING_UP(groundPos + firstLevel + increment * level, UP), ADAPTABLE_PASSTHROUGH(groundPos);

        public int position;
        public final State nextState;

        State(int position){
            this.position = position;
            this.nextState = this;
        }

        State(int position, State nextState){
            this.position = position;
            this.nextState = nextState;
        }
    }

    private State state;

    public State getState(){
        return state;
    }

    public void setState(State newState){
        updateStateValues();
        profile.setMotion(newState == State.DOWN?State.DOWN.position:encoder.getCurrentPosition(), newState.position, newState == State.DOWN?0:profile.getSignedVelocity());
        if(state == newState) return;
        this.state = newState;
    }

    private void updateStateValues(){
        State.DOWN.position = groundPos;
        State.GOING_DOWN.position = groundPos;
        State.UP.position = groundPos + firstLevel + increment * level;
        State.GOING_UP.position = groundPos + firstLevel + increment * level;
        State.GOING_PASSTHROUGH.position = passthroughPosition + groundPos;
        State.PASSTHROUGH.position = passthroughPosition + groundPos;
    }

    public Lift(Hardware hardware, State initialState){
        if(!ENABLED) leftMotor = null;
        else leftMotor = new CoolMotor(hardware.meh0, CoolMotor.RunMode.PID, leftMotorReversed);
        if(!ENABLED) rightMotor = null;
        else rightMotor = new CoolMotor(hardware.meh1, CoolMotor.RunMode.PID, rightMotorReversed);

        if(!ENABLED) encoder = null;
        else encoder = hardware.ech1;
        if(encoderReversed) encoder.setDirection(Encoder.Direction.REVERSE);

        this.state = initialState;
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateStateValues();
        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        if(state == State.RESETTING){
            if(Math.abs(encoder.getRawVelocity()) <= velocityThreshold){
                groundPos = encoder.getCurrentPosition();
                updateStateValues();
                state = state.nextState;
            }
        }
        else if(Math.abs(state.nextState.nextState.position - encoder.getCurrentPosition()) <= positionThresh)
            state = state.nextState;
    }

    public int target = 0;

    @Override
    public void updateHardware() {
        profile.update();

        if(state == State.RESETTING){
            leftMotor.setMode(CoolMotor.RunMode.RUN);
            rightMotor.setMode(CoolMotor.RunMode.RUN);
            leftMotor.setPower(resetPower);
            rightMotor.setPower(resetPower);
        }else{
            target = (int)profile.getPosition();

            leftMotor.setMode(CoolMotor.RunMode.PID);
            rightMotor.setMode(CoolMotor.RunMode.PID);
            leftMotor.setPIDF(pid, ff1 + ff2 * (double)(target));
            rightMotor.setPIDF(pid, ff1 + ff2 * (double)(target));
            leftMotor.calculatePower(encoder.getCurrentPosition(), target);
            rightMotor.calculatePower(encoder.getCurrentPosition(), target);
        }
        leftMotor.update();
        rightMotor.update();

        if(profile.finalPosition != state.position) profile.setMotion(profile.getPosition(), state.position, profile.getSignedVelocity());
    }
}
