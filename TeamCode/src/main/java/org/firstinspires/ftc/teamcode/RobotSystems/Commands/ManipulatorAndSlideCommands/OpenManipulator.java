package org.firstinspires.ftc.teamcode.RobotSystems.Commands.ManipulatorAndSlideCommands;

import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;

/**
 * The OpenManipulator command opens the robot's grabbers.
 */
public class OpenManipulator extends Commands {
    @Override
    public void run() {
        robot.manipulator.open();
    }

    @Override
    public boolean isCommandFinished() {
        return true;
    }
}
