package org.firstinspires.ftc.teamcode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Camera {
    public double x, y, z, roll, pitch, yaw;
    public int id;
    public boolean currentlyInSight = false;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    AprilTagDetection tag;

    public Camera(WebcamName camera) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public void update() {
        //REMEBER TO UPDATE THIS FOR CAMERA LATER
        currentlyInSight = false;

        x = 0;
        y = 0;
        z = 0;
        roll = 0;
        pitch = 0;
        yaw = 0;
        id = 0;

        tag = null;

        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();

//        for (AprilTagDetection tag : detections) {
//            if (tag.id == 20 || tag.id == 24) {
//                x = tag.ftcPose.x;
//                y = tag.ftcPose.y;
//                z = tag.ftcPose.z;
//                roll = tag.ftcPose.roll;
//                pitch = tag.ftcPose.pitch;
//                yaw = tag.ftcPose.yaw;
//                id = tag.id;
//
//                currentlyInSight = true;
//
//                break;
//            }
//        }
    }

    @Override
    public String toString() {
        if (currentlyInSight) {
            return "Can see tag #" + id + " at (" + roll + ", " + pitch + ", " + yaw + ")" + "at (" + x + ", " + y + ", " + z + ")";
        } else {
            return "No tag in sight";
        }
    }
}
