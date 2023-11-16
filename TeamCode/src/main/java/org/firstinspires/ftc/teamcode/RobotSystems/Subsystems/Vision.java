package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.concurrent.TimeUnit;

public class Vision {

    private Telemetry robotTelemetry = null;
    private WebcamName camera = null;
    private AprilTagProcessor aprilTagProcessor = null;
    private TfodProcessor tensorFlowProcessor = null;
    private VisionPortal visionPortal = null;
    public static final double CAMERA_OFFSET = -5.9375;
    public static final int EXPOSURE = 100;
    public static final int GAIN = 100;
    public static final int TEMPERATURE = 10;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Initialize Hardware Values
        camera = hardwareMap.get(WebcamName.class, "camera");

        // Create TensorFlow processor
        tensorFlowProcessor = TfodProcessor.easyCreateWithDefaults();

        // Create AprilTag processor
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Set up the vision portal.
        visionPortal = VisionPortal.easyCreateWithDefaults(camera, tensorFlowProcessor, aprilTagProcessor);

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

        // Tell user that the april tags were successfully initialized
        robotTelemetry.addData("->","AprilTagDetection successfully initialized.");
    }
}


