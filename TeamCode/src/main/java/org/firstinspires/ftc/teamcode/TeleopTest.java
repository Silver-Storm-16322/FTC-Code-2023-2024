package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Test", group = "Tests")
public class TeleopTest extends LinearOpMode {

    private  ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;

    @Override
    public void runOpMode() {

        // Tell the user that the robot has been initialized.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize Hardware Values
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");

        // Note: Most motors have one side reverse.
        // Due to this, we need to reverse one side of the motors so that the robot can drive straight.
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the player to press the play button.
        waitForStart();
        runtime.reset();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

            // Use some math that I 100% didn't steal to
            // calculate how the mecanum wheels will work.
            double left_stick_y = -gamepad1.left_stick_y; // (The Y is reversed, so we have to multiply it by -1)
            double left_stick_x = gamepad1.left_stick_x * 1.1; // Multiply by 1.1 to counteract imperfect straifing.
            double right_stick_x = gamepad1.right_stick_x;

            // Ensure that all powers maintain a consistent ratio.
            // This is required since all values are capped at 1.
            double denominator = Math.max(Math.abs(left_stick_y) + Math.abs(left_stick_x) + Math.abs(right_stick_x), 1);
            double rightFrontPower = (left_stick_y - left_stick_x - right_stick_x) / denominator;
            double rightBackPower = (left_stick_y + left_stick_x - right_stick_x)/ denominator;
            double leftFrontPower = (left_stick_y + left_stick_x + right_stick_x) / denominator;
            double leftBackPower = (left_stick_y - left_stick_x + right_stick_x) / denominator;

            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
        }
    }
}
