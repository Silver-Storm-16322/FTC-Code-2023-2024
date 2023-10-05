package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Robot Centric Mecanum", group="Mecanum Drive Trains 2023-2024")
public class RobotCentricMecanum extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private static DriveMode current_drive_mode = DriveMode.DEFAULT_DRIVE;
    // Settings
    private final double STRAIF_OFFSET = 1.1; // Used to help counteract imperfect straifing.


    @Override
    public void runOpMode() {

        // Tell the user that the robot has been initialized.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize Hardware Values
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");

        // Note: Most motors have one side reverse.
        // Due to this, we need to reverse one side of the motors so that the robot can drive straight.
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the player to press the play button.
        waitForStart();
        runtime.reset();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

            // Sets the drive mode to precise drive. (Goes 4* slower than default)
            // However, if the robot is already in precise drive, the it reverts to default drive.
            if (gamepad1.left_bumper) {
                current_drive_mode = updateDriveMode(DriveMode.PRECISE_DRIVE);
            }

            // Sets the drive mode to sensitive drive. (Always goes max speed)
            // However, if the robot is already in sensitive drive, the it reverts to default drive.
            if (gamepad1.right_bumper) {
                current_drive_mode = updateDriveMode(DriveMode.SENSITIVE_DRIVE);
            }

            // Get the speed multiplier of the current drive mode.
            double speedMultiplier = current_drive_mode.speedMultiplier();

            // Use some math that I 100% didn't steal to
            // calculate how the mecanum wheels will work.
            // Multiply by the speedMultiplier to make the robot go faster or slower based on the current drive mode.
            double left_stick_y = -gamepad1.left_stick_y * speedMultiplier; // (The Y is reversed, so we have to multiply it by -1)
            double left_stick_x = gamepad1.left_stick_x * STRAIF_OFFSET * speedMultiplier;
            double right_stick_x = gamepad1.right_stick_x * speedMultiplier;

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

    // Set the drive mode of the robot.
    // This determines how far the user has to move the joystick to make the robot g otop speed.
    public static DriveMode updateDriveMode(DriveMode drive_mode) {

        // Initialize local variable
        DriveMode newDriveMode;

        // Check if drive_mode is the same as the current drive mode.
        if (drive_mode == current_drive_mode) {
            newDriveMode = DriveMode.DEFAULT_DRIVE;
        } else {
            newDriveMode = drive_mode;
        }

        return newDriveMode;
    }
}
