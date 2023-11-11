package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.DriveMode;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.LinearSlideStage;
import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;

@TeleOp(name="Robot Centric Mecanum", group="Mecanum Drive Trains 2023-2024")
public class RobotCentricMecanum extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize the robot
        robot.init();

        // Wait for the driver to press the play button.
        waitForStart();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

            // Open and close the manipulator on command.
            if (gamepad1.x) {
                robot.manipulator.open();
            }
            if (gamepad1.a) {
                robot.manipulator.close();
            }

            // Sets the LinearSlide's stage to the associated stage when the dpad buttons are pressed.
            if (gamepad1.dpad_down) {
                robot.linearSlide.setStage(LinearSlideStage.GROUND_STAGE);
            }
            if (gamepad1.dpad_left) {
                robot.linearSlide.setStage(LinearSlideStage.LOW_STAGE);
            }
            if (gamepad1.dpad_up) {
                robot.linearSlide.setStage(LinearSlideStage.MID_STAGE);
            }
            if (gamepad1.dpad_right) {
                robot.linearSlide.setStage(LinearSlideStage.HIGH_STAGE);
            }

            // Sets the drive mode to precise drive. (Goes 4* slower than default)
            if (gamepad1.left_bumper) {
                robot.driveTrain.setDriveMode(DriveMode.PRECISE_DRIVE);
            }

            // Sets the drive mode to sensitive drive. (Always goes max speed)
            if (gamepad1.right_bumper) {
                robot.driveTrain.setDriveMode(DriveMode.SENSITIVE_DRIVE);
            }

            // Sets the drive mode to its default mode.
            if (gamepad1.y) {
                robot.driveTrain.setDriveMode(DriveMode.DEFAULT_DRIVE);
            }

            // These buttons are more difficult to press meaning they are less likely to be accidentally hit
            // during stressful moments.
            if (gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5) {
                robot.paperAirplaneLauncher.launchPaperAirplane();
            }

            // left_stick_y is forwards and backwards,
            // left_stick_x is left and right, and right_stick_x is the rotation
            // Note: Pass through the opposite of left stick y because it is reversed
            robot.driveTrain.driveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}