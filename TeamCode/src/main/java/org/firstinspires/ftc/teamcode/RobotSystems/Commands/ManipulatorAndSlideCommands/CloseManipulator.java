package org.firstinspires.ftc.teamcode.RobotSystems.Commands.ManipulatorAndSlideCommands;

import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;

public class CloseManipulator extends Commands {

    @Override
    public void run() {
        robot.manipulator.close();
    }

    @Override
    public boolean isCommandFinished() {
        return true;
    }
}
