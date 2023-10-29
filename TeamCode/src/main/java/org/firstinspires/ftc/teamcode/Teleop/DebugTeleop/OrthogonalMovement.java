package org.firstinspires.ftc.teamcode.Teleop.DebugTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.DriveMode;

@TeleOp (name = "Orthogonal Movement")
public class OrthogonalMovement extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize the robot
        robot.init();

        // Wait for the player to press the play button.
        waitForStart();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

            // Sets the drive mode to precise drive. (Goes 4* slower than default)
            // However, if the robot is already in precise drive, the it reverts to default drive.
            if (gamepad1.left_bumper) {
                robot.driveTrain.setDriveMode(DriveMode.PRECISE_DRIVE);
            }

            // Sets the drive mode to sensitive drive. (Always goes max speed)
            // However, if the robot is already in sensitive drive, the it reverts to default drive.
            if (gamepad1.right_bumper) {
                robot.driveTrain.setDriveMode(DriveMode.SENSITIVE_DRIVE);
            }

            // Determine which axis has more force put into it.
            double xMove = 0;
            double yMove = 0;
            if (Math.abs(-gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) {
                yMove = -gamepad1.left_stick_y;
                xMove = 0;
            } else {
                xMove = gamepad1.left_stick_x;
                yMove = 0;
            }

            // left_stick_y is forwards and backwards,
            // left_stick_x is left and right, and right_stick_x is the rotation
            // Note: Pass through the opposite of left stick y because it is reversed
            robot.driveTrain.driveRobot(yMove, xMove, gamepad1.right_stick_x);
        }
    }
}
