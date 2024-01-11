package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Wrappers.EncoderFunny;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889763779527559; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 2.8529921; // X is the up and down direction
    public static double PARALLEL_Y = 3.937007; // Y is the strafe direction

    public static double PERPENDICULAR_X = 3.186456;
    public static double PERPENDICULAR_Y = -3.83771;

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    public static boolean parallelEncoderReversed = false, perpendicularEncoderReversed = true;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private final Encoder parallelEncoder, perpendicularEncoder;

    private final CoolIMU imu;

    public TwoWheelTrackingLocalizer(Hardware hardware) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.imu = hardware.imu;

        parallelEncoder = hardware.ech3;
        perpendicularEncoder = hardware.ech0;

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        if(parallelEncoderReversed) parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        if(perpendicularEncoderReversed) perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imu.getHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return imu.getVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
