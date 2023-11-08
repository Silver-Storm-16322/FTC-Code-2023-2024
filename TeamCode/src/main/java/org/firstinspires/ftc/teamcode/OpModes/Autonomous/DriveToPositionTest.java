package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vector3;

@Autonomous (name = "DriveToPositionTest", group = "Debug OpModes")
public class DriveToPositionTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() {

        // Initialize all of the robot's hardware.
        robot.init();

        // Wait for the driver to press the play button.
        waitForStart();

        // Repeatedly drive the robot around to the set positions to test the robot's coordinate system
        // and driveToPosition method.
        while (opModeIsActive()) {

            // Drive the robot to a bunch op pre-planned positions (in inches) //
            robot.driveTrain.driveRobotToPosition(new Vector3(0, 0, 0));
            robot.driveTrain.driveRobotToPosition(new Vector3(14, 0, 0));
            robot.driveTrain.driveRobotToPosition(new Vector3(0, 14, 0));
            robot.driveTrain.driveRobotToPosition(new Vector3(-14, 0, 0));
            robot.driveTrain.driveRobotToPosition(new Vector3(0, -14, 0));
        }
    }
}
