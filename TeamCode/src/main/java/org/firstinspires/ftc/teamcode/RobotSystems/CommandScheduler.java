package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutoParams;

import java.util.ArrayList;
import java.util.List;

public class CommandScheduler {
    private List<Commands> commands = new ArrayList<Commands>();
    private RobotHardware robot = null;
    private LinearOpMode myOpMode = null;
    private AutoParams autoParams = null;

    /**
     * Initializes the CommandScheduler with the robot hardware and telemetry.
     *
     * @param robot     The robot's hardware configuration.
     */
    public CommandScheduler(RobotHardware robot, LinearOpMode opMode, AutoParams autoParams) {
        this.robot = robot;
        this.myOpMode = opMode;
        this.autoParams = autoParams;
    }

    /**
     * Adds a command to the list of commands that will be executed in order and then provides the
     * command with all of the data it needs to be able function properly.
     *
     * @param command The command to be added to the scheduler.
     */
    public void addCommand(Commands command) {
        commands.add(command);
        command.init(robot, myOpMode.telemetry, autoParams);
    }

    /**
     * Executes all commands in the scheduler sequentially.
     */
    public void run() {

        // Ensure there are commands to run.
        if (commands.size() == 0) {
            return;
        }

        // Loop through and execute all commands in the scheduler.
        for (Commands command : commands) {

            // Run the command.
            command.run();

            // Wait until the command has finished running.
            while (myOpMode.opModeIsActive() && !command.isCommandFinished()) {}
        }
    }
}
