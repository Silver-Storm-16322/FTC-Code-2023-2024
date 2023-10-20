package org.firstinspires.ftc.teamcode.Teleop.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotSystem.Vision;

@TeleOp(name="Drive To April Tags Test", group="System Tests")
public class DriveToAprilTagTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    Vision aprilTagDetection = new Vision(this);
    @Override
    public void runOpMode() {

        // Initialize the robot and april tags.
        aprilTagDetection.init();
        robot.init();

        // Wait for the player to press the play button.
        waitForStart();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

        }
    }
}