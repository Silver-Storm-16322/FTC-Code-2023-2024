package org.firstinspires.ftc.teamcode.OpModes.CalibrationOpModes.PIDCalibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

@TeleOp (name = "TurnToPIDTuner", group = "Debug OpModes")
public class TurnToPIDTuner extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the robot
        robot.init();

        // Wait for the player to press the play button.
        waitForStart();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(0, 0, 0));
            }
            if (gamepad1.dpad_right) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(0, 0, 90));
            }
            if (gamepad1.dpad_down) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(0, 0, 180));
            }
            if (gamepad1.dpad_left) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(0,0 ,-90));
            }
        }
    }
}
