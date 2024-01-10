package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.AsymmetricMotionProfile;

public class CoolServo {

    private final ServoFunny servo;
    public AsymmetricMotionProfile profile;
    private boolean isProfiled = false;

    private final boolean async;

    public CoolServo(Servo servo, boolean reversed, double initialPosition){
        this.servo = new ServoFunny(servo);
        this.async = false;
        if(reversed) this.servo.servo.setDirection(Servo.Direction.REVERSE);
        setInitialPosition(initialPosition);
    }

    public CoolServo(Servo servo, boolean reversed, double profileMaxVelocity, double profileAcceleration, double profileDeceleration, double initialPosition){
        this.servo = new ServoFunny(servo);
        this.async = false;
        if(reversed) this.servo.servo.setDirection(Servo.Direction.REVERSE);
        profile = new AsymmetricMotionProfile(profileMaxVelocity, profileAcceleration, profileDeceleration);
        isProfiled = true;
        setInitialPosition(initialPosition);
    }

    public CoolServo(Servo servo, boolean reversed, double profileMaxVelocity, double profileAcceleration, double initialPosition){
        this(servo, reversed, profileMaxVelocity, profileAcceleration, profileAcceleration, initialPosition);
    }

    public CoolServo(ServoFunny servo, boolean reversed, double initialPosition){
        this.servo = servo;
        this.async = true;
        if(reversed) this.servo.servo.setDirection(Servo.Direction.REVERSE);
        setInitialPosition(initialPosition);
    }

    public CoolServo(ServoFunny servo, boolean reversed, double profileMaxVelocity, double profileAcceleration, double profileDeceleration, double initialPosition){
        this.servo = servo;
        this.async = true;
        if(reversed) this.servo.servo.setDirection(Servo.Direction.REVERSE);
        profile = new AsymmetricMotionProfile(profileMaxVelocity, profileAcceleration, profileDeceleration);
        isProfiled = true;
        setInitialPosition(initialPosition);
    }

    public CoolServo(ServoFunny servo, boolean reversed, double profileMaxVelocity, double profileAcceleration, double initialPosition){
        this(servo, reversed, profileMaxVelocity, profileAcceleration, profileAcceleration, initialPosition);
    }

    public double cachedPosition, targetPosition;

    private void setInitialPosition(double pos){
        cachedPosition = pos;
        targetPosition = pos;
        if(isProfiled)profile.setMotion(pos, pos, 0);
    }

    public void setPosition(double position){
        if(position == targetPosition) return;
        targetPosition = position;
        if(isProfiled) profile.setMotion(cachedPosition, targetPosition, profile.getSignedVelocity());
    }

    public void update(){
        if(isProfiled) profile.update();
        if(isProfiled && cachedPosition != profile.getPosition()) {
            cachedPosition = profile.getPosition();
            servo.setPositionAsync(cachedPosition);
        }
        if(!isProfiled && cachedPosition != targetPosition) {
            cachedPosition = targetPosition;
            servo.setPositionAsync(targetPosition);
        }
        if(!async) servo.updatePositionAsync();
    }

    public void forceUpdate(){
        if(isProfiled) profile.update();
        if(isProfiled) {
            cachedPosition = profile.getPosition();
            servo.setPositionAsync(cachedPosition);
        }
        if(!isProfiled) {
            cachedPosition = targetPosition;
            servo.setPositionAsync(targetPosition);
        }
        if(!async) servo.updatePositionAsync();
    }

    public boolean isProfiled() {
        return isProfiled;
    }

    public double getTimeToMotionEnd(){
        if(!isProfiled) return 0;
        return profile.getTimeToMotionEnd();
    }
}
