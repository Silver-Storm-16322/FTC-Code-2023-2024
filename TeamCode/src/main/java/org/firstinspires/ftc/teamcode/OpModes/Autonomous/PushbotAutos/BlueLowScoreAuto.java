package org.firstinspires.ftc.teamcode.OpModes.Autonomous.PushbotAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

@Autonomous
public class BlueLowScoreAuto extends LinearOpMode {

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
            robot.driveTrain.driveRobotToPosition(new RobotPosition(5, 0, 0));
            robot.driveTrain.driveRobotToPosition(new RobotPosition(5, 35, 0));
            robot.driveTrain.driveRobotToPosition(new RobotPosition(5, 35, -90));

            sleep(90000000);
        }
    }
}