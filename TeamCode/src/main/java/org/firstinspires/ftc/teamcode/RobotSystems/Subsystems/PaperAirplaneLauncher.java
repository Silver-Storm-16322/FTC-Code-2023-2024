package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PaperAirplaneLauncher {
    private Servo airplaneLauncher = null;
    public static final double LAUNCH_POSITION = 1.0;
    public static final double IDLE_POSITION = 0;
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Initialize Hardware Variables //
        airplaneLauncher = hardwareMap.get(Servo.class, "paperAirplaneLauncher");

        // Configure Servo //
        airplaneLauncher.setDirection(Servo.Direction.REVERSE);

        // Tell user the PaperAirplaneLauncher has been successfully initialized.
        telemetry.addData("->","PaperAirplaneLauncher successfully initialized");
    }

    public void launchPaperAirplane() {
        airplaneLauncher.setPosition(LAUNCH_POSITION);
    }
}
