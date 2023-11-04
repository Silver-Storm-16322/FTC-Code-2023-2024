package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class Vision {

    private Telemetry robotTelemetry = null;
    private WebcamName webcam = null;
    private AprilTagProcessor tagProcessor = null;
    private VisionPortal visionPortal = null;
    public static final double CAMERA_OFFSET = -5.9375;


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Initialize Hardware Values
        webcam = hardwareMap.get(WebcamName.class, "camera");

        // Initialize the tagProcesses that will
        tagProcessor = new AprilTagProcessor.Builder()

                .setDrawAxes(true)                                                  // Shows the tag's axis.
                .setDrawCubeProjection(true)                                        // Draws a cube off of the april tag to better se where the camera thinks the tag is pointing.
                .setDrawTagID(true)                                                 // Shows the ID of the april tag.
                .setDrawTagOutline(true)                                            // Draws an outline around the april tag.

                .build();

        visionPortal = new VisionPortal.Builder()

                .addProcessor(tagProcessor)                                         // Add the tag processor here
                .setCamera(webcam)                                                  // Add the camera on the robot here.
                .setCameraResolution(new Size(640, 480))               // Higher res, sees farther.uses more CPU
                .enableLiveView(true)                                               // Allows the camera to display what it sees on the driver hub.

                .build();

        // Wait until the cameras-tarts streaming. This is done because if we adjust the camera's
        // values before it starts streaming, then the opMode will crash.
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        // Create exposure and apply it to the camera
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        // Create gain and apply it to the camera.
        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);

        // Allows this drivetrain to communicate with the user.
        robotTelemetry = telemetry;

        // Tell user that the april tags were successfully initialized
        robotTelemetry.addData("->","AprilTagDetection successfully initialized.");
    }

    public boolean aprilTagVisible() {
        return tagProcessor.getDetections().size() != 0;
    }

    public org.firstinspires.ftc.vision.apriltag.AprilTagDetection getClosestTagWithPositionData() {

        double closestTagDistance = 9999;
        org.firstinspires.ftc.vision.apriltag.AprilTagDetection closestTag = null;

        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection tag : tagProcessor.getDetections()) {

            if (tag.ftcPose == null) { continue; }

            if (tag.ftcPose.range < closestTagDistance) {
                closestTagDistance = tag.ftcPose.range;
                closestTag = tag;
            }
        }

        return closestTag;
    }

    public AprilTagDetection getTagWithId(int specifiedTagId) {
        for (AprilTagDetection tag : tagProcessor.getDetections()) {
            if (tag.id == specifiedTagId) {
                return tag;
            }
        }
        return null;
    }
}


