package org.firstinspires.ftc.teamcode.Teleop.TestFiles;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@TeleOp (name = "April tags test", group = "System Tests")
public class Vision extends LinearOpMode {

    private WebcamName webcam = null;
    @Override
    public void runOpMode() {

        // Initialize Hardware
        webcam = hardwareMap.get(WebcamName.class, "camera");

        // Initialize April tag variables
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)                  // Shows the tag's axis.
                .setDrawCubeProjection(true)        // Draws a cube off of the april tag to better se where the camera thinks the tag is pointing.
                .setDrawTagID(true)                 // Shows the ID of the april tag.
                .setDrawTagOutline(true)            // Draws an outline around the april tag.
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)         // Add the tag processor here
                .setCamera(webcam)                  // Add the camera on the robot here.
                .setCameraResolution(new Size(640, 480)) // Higher res, sees farther.uses more CPU
                .enableLiveView(true)
                .build();

        // Wait until the camera starts streaming (Opcode crashes if we attempt to change exposure and other values before the camera is streaming)
       while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

       ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
       exposure.setMode(ExposureControl.Mode.Manual);
       exposure.setExposure(15, TimeUnit.MILLISECONDS);

       GainControl gain = visionPortal.getCameraControl(GainControl.class);
       gain.setGain(255);

       // Wait for the start of the opMode
       waitForStart();

        while (opModeIsActive()) {

            // Checks if the camera detects an april tag
            // Tag is equal to the first detected tag unless n tag is detected.
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                /*
                telemetry.addData("exposure", exposure.isExposureSupported());

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("elevation", tag.ftcPose.elevation);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("roll", tag.ftcPose.roll);
                */
            }

            telemetry.update();
        }
    }
}
