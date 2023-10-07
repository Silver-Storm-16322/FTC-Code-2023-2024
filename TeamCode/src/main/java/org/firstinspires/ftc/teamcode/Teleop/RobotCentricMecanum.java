package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot Centric Mecanum", group="Mecanum Drive Trains 2023-2024")
public class RobotCentricMecanum extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() {

        // Initialize the robot
        robot.init();

        // Wait for the player to press the play button.
        waitForStart();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

            /*
            * Sets the drive mode to precise drive. (Goes 4* slower than default)
            * However, if the robot is already in precise drive, the it reverts to default drive.
            */
            if (gamepad1.left_bumper) {
                robot.setDriveMode(DriveMode.PRECISE_DRIVE);
            }

            /*
            * Sets the drive mode to sensitive drive. (Always goes max speed)
            * However, if the robot is already in sensitive drive, the it reverts to default drive.
            */
            if (gamepad1.right_bumper) {
                robot.setDriveMode(DriveMode.SENSITIVE_DRIVE);
            }

            /*
            * left_stick_y is forwards and backwards,
            * left_stick_x is left and right, and right_stick_x is the rotation
            * Note: Pass through the opposite of left stick y because it is reversed
             */
            robot.driveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}