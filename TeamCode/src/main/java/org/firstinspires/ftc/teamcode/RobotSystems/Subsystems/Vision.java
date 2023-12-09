package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {

    private LinearOpMode myOpMode = null;
    private WebcamName camera = null;
    private AprilTagProcessor aprilTagProcessor = null;
    private TfodProcessor tensorFlowProcessor = null;
    private VisionPortal visionPortal = null;
    public static final double CAMERA_OFFSET = -5.9375;

    // Tensor Flow Settings //
    private static final String MODE_NAME = "TeamPropDetection.tflite";
    private static final String[] LABELS = {
            "BlueTeamProp",
            "RedTeamProp"
    };
    // Camera Settings
    public static final int EXPOSURE = 100;
    public static final int GAIN = 100;
    public static final int TEMPERATURE = 10;

    public Vision(LinearOpMode opMode) {this.myOpMode = opMode; }
    public void init() {

        // Initialize Hardware Values
        camera = myOpMode.hardwareMap.get(WebcamName.class, "camera");

        // Create TensorFlow processor
        tensorFlowProcessor = new TfodProcessor.Builder()
                .setModelAssetName(MODE_NAME)
                .setModelLabels(LABELS)
                .build();


        // Create AprilTag processor
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Set up the vision portal.
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .addProcessors(tensorFlowProcessor, aprilTagProcessor)
                .build();

        // Wait until the cameras-tarts streaming. This is done because if we adjust the camera's
        // values before it starts streaming, then the opMode will crash.
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.idle();
        }

        /* -- Camera Calibration Stuff --

        // Setup gain and exposure.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        // Get Maximum and minimum gain and exposure values.
        double minExposure = exposureControl.getMinExposure(TimeUnit.MICROSECONDS);
        double maxExposure = exposureControl.getMaxExposure(TimeUnit.MICROSECONDS);
        double minGain = gainControl.getMinGain();
        double maxGain   = gainControl.getMaxGain();

        // Set exposure mode to manual. (Also sets manual gain)
        exposureControl.setMode(ExposureControl.Mode.Manual);

        // Set exposure and gain values.
        exposureControl.setExposure(EXPOSURE, TimeUnit.MICROSECONDS);
        gainControl.setGain(GAIN);

        // Setup white balance control methods.
        WhiteBalanceControl wbControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        // White balance control
        int minTemperature = wbControl.getMinWhiteBalanceTemperature();
        int maxTemperature = wbControl.getMaxWhiteBalanceTemperature();

        // Set balance mode to manual.
        wbControl.setMode(WhiteBalanceControl.Mode.MANUAL);

        // Set temperature.
        wbControl.setWhiteBalanceTemperature(TEMPERATURE);
         */

        // Tell user that the april tags were successfully initialized
        myOpMode.telemetry.addData("->","Vision successfully initialized.");
    }

    /**
     * This method checks if the camera can see any team props. If it can, then we determine which
     * object the AI has the most confidence in being the team prop. Otherwise, return null.
     *
     * @param allianceMultiplier The alliance multiplier specified by the autoParams object.
     * @return Returns the alliance team prop that the AI has the most confidence in.
     */
    public Recognition getDetectedTeamProps(double allianceMultiplier) {

        // Create a value to store the team prop that the AI has the most confidence in.
        Recognition detectedTeamProp = null;
        double highestDetectedObjectConfidence = 0;

        // Get a list containing all of the objects that the camera is detecting.
        List <Recognition> detectedTeamProps = tensorFlowProcessor.getRecognitions();

        /*
        Based on the provided multiplier value, determine what team prop to look for.
        1 is RedTeamProp
        -1 is BlueTeamProp
         */
        String alliancePropName = (allianceMultiplier == 1) ? "RedTeamProp" : "BlueTeamProp";

        // Loop through all of the detected objects and determine which one the AI thinks is most likely
        // to be the team prop.
        for (Recognition detectedObject : detectedTeamProps) {

            // Check if detectedObject has been labeled as the alliance specific team prop.
            // if not, then skip to the next object.
            if (!detectedObject.getLabel().equals(alliancePropName)) {
                continue;
            }

            /*
            Check if the AI has more confidence that this object is our team prop than the previously
            detected object with the most confidence. If it is, then set it as the detected object
            and set highestDetectedObjectConfidence to the detected object's confidence.
             */
            if (detectedObject.getConfidence() > highestDetectedObjectConfidence) {
                detectedTeamProp = detectedObject;
                highestDetectedObjectConfidence = detectedObject.getConfidence();
            }
        }

        // Return the team prop that the AI has the most confidence in.
        return detectedTeamProp;
    }
}


