package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystems.CommandScheduler;
import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;

@Autonomous(name = "Red-MediumScore-Autonomous", group = "Red", preselectTeleOp = "Robot Centric Mecanum")
public class RedMediumScoreAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);;
    FieldLocation startingLocation = FieldLocation.RED_BACKDROP;
    CommandScheduler scoreYellowAndPurplePixels = new CommandScheduler(robot, telemetry);
    CommandScheduler getPixelsAndPlaceThemOnBackdrop = new CommandScheduler(robot, telemetry);

    @Override
    public void runOpMode() {

        // Initialize RobotHardware
        robot.init();


        // Wait for the OpMode to be started.
        waitForStart();
    }
}
