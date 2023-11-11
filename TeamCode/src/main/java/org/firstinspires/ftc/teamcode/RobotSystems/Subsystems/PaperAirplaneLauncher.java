package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PaperAirplaneLauncher {
    private LinearOpMode myOpMode = null;
    private Servo airplaneLauncher = null;
    public static final double LAUNCH_POSITION = 1.0;
    public static final double IDLE_POSITION = -1.0;

    public PaperAirplaneLauncher(LinearOpMode opMode) { this.myOpMode = opMode; }
    public void init() {

        // Initialize Hardware Variables //
        airplaneLauncher = myOpMode.hardwareMap.get(Servo.class, "paperAirplaneLauncher");

        // Configure Servo //
        airplaneLauncher.setDirection(Servo.Direction.REVERSE);

        // Tell user the PaperAirplaneLauncher has been successfully initialized.
        myOpMode.telemetry.addData("->","PaperAirplaneLauncher successfully initialized");
    }

    /**
     * Move the servo to a position where the paper airplane is launched into the air.
     * Then, after the paper airplane has been launched move the servo back to it's starting position.
     */
    public void launchPaperAirplane() {

        // Move the servo to the position that launches the paper airplane.
        airplaneLauncher.setPosition(LAUNCH_POSITION);

        // Waits until either the opMode is stopped, or the paper airplane launcher is launched.
        while (myOpMode.opModeIsActive() && airplaneLauncher.getPosition() != LAUNCH_POSITION) {
            myOpMode.idle();
        }

        // Reset the servo back to it's idle position so that another paper airplane can be loaded
        // when the round is over. (Saves a bunch of time :D)
        resetPaperAirplaneLauncher();
    }

    /**
     * Moves the servo back to the position where a paper airplane can be loaded in.
     */
    public void resetPaperAirplaneLauncher() {
        airplaneLauncher.setPosition(IDLE_POSITION);
    }
}
