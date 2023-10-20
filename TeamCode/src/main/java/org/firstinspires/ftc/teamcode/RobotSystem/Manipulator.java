package org.firstinspires.ftc.teamcode.RobotSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {

    private LinearOpMode myOpMode = null;
    private Servo rightGripperServo = null;
    private Servo leftGripperServo = null;
    public static final int CLOSED_POSITION = 0;
    public static final int OPEN_POSITION = 45;

    // Get the opMode so that we can get hardware.
    public Manipulator(LinearOpMode opMode) {myOpMode = opMode; }

    public void init() {

        // Initialize Hardware Variables.
        rightGripperServo = myOpMode.hardwareMap.get(Servo.class, "rightGripperServo");
        leftGripperServo = myOpMode.hardwareMap.get(Servo.class, "leftGripperServo");

        // Reverse movement direction of one of the servos so we can use the same value for positions for both servos.
        rightGripperServo.setDirection(Servo.Direction.REVERSE);
        leftGripperServo.setDirection(Servo.Direction.FORWARD);

        // Tell the user that the manipulator has been initialized.
        myOpMode.telemetry.addData("-->","Manipulator initialized");
        myOpMode.telemetry.update();
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
