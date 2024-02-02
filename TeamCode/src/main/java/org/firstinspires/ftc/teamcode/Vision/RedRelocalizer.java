package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class RedRelocalizer {
    AprilTagProcessor processor;
    Localizer localizer;

    public RedRelocalizer(AprilTagProcessor processor, Localizer localizer){
        this.processor = processor;
        this.localizer = localizer;
    }

    public static int correctID = 4;

    public static double xOffset = 29.8, yOffset = -91.5;

    public double lastX, lastY;
    public double distance, globalAngle;

    private Pose getPoseFromTag(AprilTagDetection tag){
        distance = Math.sqrt(tag.ftcPose.x * tag.ftcPose.x + tag.ftcPose.y * tag.ftcPose.y);
        globalAngle = tag.ftcPose.yaw + localizer.getHeading();

        double xDistance = -distance * Math.cos(globalAngle);
        double yDistance = -distance * Math.sin(globalAngle);

        lastX = xDistance + xOffset;
        lastY = yDistance + yOffset;

        return new Pose(xDistance + xOffset, yDistance + yOffset, localizer.getHeading());
    }

    public long frameAcquisitionTime = 0;
    public boolean updated = false;

    public boolean use = false;

    public void update(){
        if(!use) return;
        for(AprilTagDetection tag: processor.getDetections()){
            if(tag.id == correctID){
                if(frameAcquisitionTime != tag.frameAcquisitionNanoTime){
                    frameAcquisitionTime = tag.frameAcquisitionNanoTime;
                    updated = true;
                    localizer.setPose(getPoseFromTag(tag));
//                    Pose a = getPoseFromTag(tag);
                }
            }
        }
    }

}
