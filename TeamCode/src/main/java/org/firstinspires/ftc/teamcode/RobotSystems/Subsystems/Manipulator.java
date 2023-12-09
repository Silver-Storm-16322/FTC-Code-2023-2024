package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Manipulator {

    private Telemetry robotTelemetry = null;
    public Servo rightGripperServo = null;
    public Servo leftGripperServo = null;
    public static final double CLOSED_POSITION = 0;
    public static final double OPEN_POSITION = .625;

    // Get the opMode so that we can get hardware.
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Initialize Hardware Variables.
        rightGripperServo = hardwareMap.get(Servo.class, "rightGripperServo");
        leftGripperServo = hardwareMap.get(Servo.class, "leftGripperServo");

        // Reverse movement direction of one of the servos so we can use the same value for
        // positions for both servos.
        rightGripperServo.setDirection(Servo.Direction.REVERSE);
        leftGripperServo.setDirection(Servo.Direction.FORWARD);

        // Allow this subsystem to communicate with the user.
        robotTelemetry = telemetry;

        // Tell the user that the manipulator has been initialized.
        robotTelemetry.addData("->","Manipulator initialized");
    }

    // Open up the manipulator so that we can grab a hexagon.
    public void open() {
        rightGripperServo.setPosition(OPEN_POSITION + .05);
        leftGripperServo.setPosition(OPEN_POSITION);
    }

    // Close the manipulator so that we can move/manipulate a hexagon.
    public void close() {
        rightGripperServo.setPosition(CLOSED_POSITION + .1);
        leftGripperServo.setPosition(CLOSED_POSITION);
    }

    public boolean isPixelGrabbed() {
        return false;
    }
}
