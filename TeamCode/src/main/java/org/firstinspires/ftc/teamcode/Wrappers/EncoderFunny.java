package org.firstinspires.ftc.teamcode.Wrappers;

public class EncoderFunny {
    public Encoder encoder;
    private int position;
    private double velocity, correctedVelocity;
    final Object lock = new Object();

    public EncoderFunny(Encoder encoder){
        this.encoder = encoder;
    }

    public void read(){
        synchronized (lock){
            this.position = encoder.getCurrentPosition();
            this.velocity = encoder.getRawVelocity();
            this.correctedVelocity = encoder.getCorrectedVelocity();
        }
    }

    public void setReversed(){
        encoder.setDirection(Encoder.Direction.REVERSE);
    }

    public int getCurrentPosition(){
        return position;
    }

    public double getCorrectedVelocity(){
        return correctedVelocity;
    }

    public double getVelocity(){
        return velocity;
    }
}
