package org.firstinspires.ftc.teamcode.RobotSystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutoParams;

import java.util.ArrayList;
import java.util.List;

public class CommandScheduler {
    private List<Commands> commands = new ArrayList<Commands>();
    private RobotHardware robot;
    private Telemetry telemetry;
    private AutoParams autoParams;

    /**
     * Initializes the CommandScheduler with the robot hardware and telemetry.
     *
     * @param robot     The robot's hardware configuration.
     * @param telemetry The telemetry instance for logging information.
     */
    public CommandScheduler(RobotHardware robot, Telemetry telemetry, AutoParams autoParams) {
        this.robot = robot;
        this.telemetry = telemetry;
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
        command.init(robot, telemetry, autoParams);
    }

    /**
     * Returns the number of commands remaining in the scheduler.
     *
     * @return The number of commands still to be executed.
     */
    public int getListLength() {
        return commands.size();
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
            while (!command.isCommandFinished()) {}
        }
    }
}
