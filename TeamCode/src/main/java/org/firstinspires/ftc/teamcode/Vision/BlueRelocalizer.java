package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class BlueRelocalizer {
    AprilTagProcessor processor;
    Localizer localizer;

    public BlueRelocalizer(AprilTagProcessor processor, Localizer localizer){
        this.processor = processor;
        this.localizer = localizer;
    }

    public static int correctID = 3;

    public static double xOffset = 28, yOffset = 91;

    private Pose getPoseFromTag(AprilTagDetection tag){
        double distance = Math.sqrt(tag.ftcPose.x * tag.ftcPose.x + tag.ftcPose.y * tag.ftcPose.y);
        double globalAngle = tag.ftcPose.yaw + localizer.getHeading();

        double xDistance = -distance * Math.cos(globalAngle);
        double yDistance = -distance * Math.sin(globalAngle);

        return new Pose(xDistance + xOffset, yDistance + yOffset, localizer.getHeading());
    }

    private long frameAcquisitionTime = 0;

    public void update(){
        for(AprilTagDetection tag: processor.getDetections()){
            if(tag.id == correctID){
                if(frameAcquisitionTime != tag.frameAcquisitionNanoTime){
                    frameAcquisitionTime = tag.frameAcquisitionNanoTime;
                    localizer.setPose(getPoseFromTag(tag));
                }
            }
        }
    }

}