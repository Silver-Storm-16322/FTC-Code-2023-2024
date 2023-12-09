package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystems.CommandScheduler;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.ScoreTeamPropPixels;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.SetupHardware;
import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutoParams;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums.PixelStack;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums.StartingLocation;

@Autonomous(name = "Red-MediumScore-Autonomous", group = "Red", preselectTeleOp = "Robot Centric Mecanum")
public class MediumScoreAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);;
    AutoParams autoParams = new AutoParams(StartingLocation.RED_WING, PixelStack.RED_SIDE_STACK_1);
    int parkingLocation = 0;
    @Override
    public void runOpMode() {

        // Initialize RobotHardware
        robot.init();

        // Wait for the OpMode to be started.
        while (opModeInInit()) {

            // Sets the robot's starting location to the specified locating when each button on the
            // dpad is pressed.
            if (gamepad1.dpad_up) {
                autoParams.setStartingLocation(StartingLocation.RED_BACKDROP);
            }
            if (gamepad1.dpad_down) {
                autoParams.setStartingLocation(StartingLocation.RED_WING);
            }
            if (gamepad1.dpad_left) {
                autoParams.setStartingLocation(StartingLocation.BLUE_WING);
            }
            if (gamepad1.dpad_right) {
                autoParams.setStartingLocation(StartingLocation.BLUE_BACKDROP);
            }

            if (gamepad1.a) {
                parkingLocation = 1; // Left side of backdrop
            }
            if (gamepad1.b) {
                parkingLocation = 0; // Right side of backdrop
            }

            // Display the current auto params so that the driver can make changes if needed.
            telemetry.addLine("---| Auto Settings |---");
            telemetry.addData("Starting Location: ", autoParams.getStartingLocation());
            telemetry.addData("Parking Location ", (parkingLocation == 0) ? "Right" : "Left");

            telemetry.update();
        }

        // Create the command list
        CommandScheduler commandList = new CommandScheduler(robot, this, autoParams);

        // initialize commands
        commandList.addCommand(new SetupHardware());
        commandList.addCommand(new ScoreTeamPropPixels(true, parkingLocation,this));

        while (opModeIsActive()) {

            // Run the commands
            commandList.run();
            sleep(99999999);
        }
    }
}
