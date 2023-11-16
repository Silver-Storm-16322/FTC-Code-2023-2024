package org.firstinspires.ftc.teamcode.OpModes.Teleop.DebugTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;

@TeleOp(name = "PaperAirplaneLauncherTest", group = "Debug OpModes")

public class PaperAirplaneLauncherTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize the robot
        robot.init();

        // Wait for the driver to press the play button.
        waitForStart();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

            // These buttons are more difficult to press meaning they are less likely to be accidentally hit
            // during stressful moments.
            if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {
                robot.paperAirplaneLauncher.launchPaperAirplane();
            }
        }
    }
}
