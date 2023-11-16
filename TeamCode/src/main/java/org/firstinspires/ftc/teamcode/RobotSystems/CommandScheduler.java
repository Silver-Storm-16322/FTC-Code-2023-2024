package org.firstinspires.ftc.teamcode.RobotSystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;

import java.util.ArrayList;
import java.util.List;

public class CommandScheduler {
    private List<Commands> commands = new ArrayList<Commands>();
    private RobotHardware robot = null;
    private Telemetry telemetry = null;

    /**
     * Initializes the CommandScheduler with the robot hardware and telemetry.
     *
     * @param robot     The robot's hardware configuration.
     * @param telemetry The telemetry instance for logging information.
     */
    public CommandScheduler(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    /**
     * Adds a command to the list of commands that will be executed in order.
     *
     * @param command The command to be added to the scheduler.
     */
    public void addCommand(Commands command) {
        commands.add(command);
        command.init(robot, telemetry);
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
            command.run();
        }
    }
}
