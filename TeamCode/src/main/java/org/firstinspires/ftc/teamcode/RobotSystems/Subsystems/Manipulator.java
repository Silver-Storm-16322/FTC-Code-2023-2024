package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Manipulator {

    private Telemetry robotTelemetry = null;
    private Servo rightGripperServo = null;
    private Servo leftGripperServo = null;
    public static final double CLOSED_POSITION = 0.5;
    public static final double OPEN_POSITION = 0.1;

    // Get the opMode so that we can get hardware.
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Initialize Hardware Variables.
        rightGripperServo = hardwareMap.get(Servo.class, "rightGripperServo");
        leftGripperServo = hardwareMap.get(Servo.class, "leftGripperServo");

        // Reverse movement direction of one of the servos so we can use the same value for positions for both servos.
        rightGripperServo.setDirection(Servo.Direction.REVERSE);
        leftGripperServo.setDirection(Servo.Direction.FORWARD);

        // Allow this subsystem to communicate with the user.
        robotTelemetry = telemetry;

        // Tell the user that the manipulator has been initialized.
        robotTelemetry.addData("->","Manipulator initialized");
    }

    // Open up the manipulator so that we can grab a hexagon.
    public void open() {
        rightGripperServo.setPosition(OPEN_POSITION);
        leftGripperServo.setPosition(OPEN_POSITION);
    }

    // Close the manipulator so that we can move the game piece.
    public void close() {
        rightGripperServo.setPosition(CLOSED_POSITION);
        leftGripperServo.setPosition(CLOSED_POSITION);
    }
}
