package org.firstinspires.ftc.teamcode.Teleop.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Linear Slide text", group = "System Tests")
public class Linear_Slide_Test extends LinearOpMode {

    // Settings //
    private static final int MAX_COUNT = 50000;
    private static final int MIN_COUNT = 0;
    private int targetPosition = MIN_COUNT;

    @Override
    public void runOpMode() {

        // Initialize Hardware Values
        DcMotor lsMotor = hardwareMap.get(DcMotor.class, "linear_slide_motor");

        // Configure Motors
        lsMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsMotor.setTargetPosition(targetPosition); // We have to set a target position before using run to position to prevent the code from breaking.
        lsMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the user that the robot has been initialized
        telemetry.addData("Status","Initialized");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            // Increase the linear slide position by 5 (if possible) when the down button on the dpad is pressed.
            if (gamepad1.dpad_up) {
                targetPosition = adjustTargetPos(targetPosition, 5);
                lsMotor.setTargetPosition(targetPosition);
            }

            // Decrease the linear slide position by 5 (if possible) when the down button on the dpad is pressed.
            if (gamepad1.dpad_down) {
                targetPosition = adjustTargetPos(targetPosition, -5);
                lsMotor.setTargetPosition(targetPosition);
            }

            // Display Current Linear Slide Status
            telemetry.addLine("//-----// Linear Slide Status //-----//");
            telemetry.addData("Current Position", lsMotor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }
    }

    // Adjusts the linear slide's position by +-5 if it wont be less then MIN_COUNT and wont be greater than MAX_COUNT
    public static int adjustTargetPos(int position, int change) {

        // Initialize local variables.
        int newPosition =  position + change;

        // Make sure the adjusted Linear Slide position is in between the maximum and minimum values.
        if (newPosition > MAX_COUNT) {
            newPosition = MAX_COUNT;
        } else if (newPosition < MIN_COUNT) {
            newPosition = MIN_COUNT;
        }

        return newPosition;
    }
}
