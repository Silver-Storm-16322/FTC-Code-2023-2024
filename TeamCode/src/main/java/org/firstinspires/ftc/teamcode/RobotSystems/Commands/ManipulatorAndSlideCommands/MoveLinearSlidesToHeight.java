package org.firstinspires.ftc.teamcode.RobotSystems.Commands.ManipulatorAndSlideCommands;

import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;

public class MoveLinearSlidesToHeight extends Commands {
    double height = 0;

    /**
     * Tells the command how high to raise the linear slides.
     *
     * @param height The height (in inches) that you want to raise the linear slides to.
     */
    public MoveLinearSlidesToHeight(double height) {
        this.height = height;
    }

    /**
     * Raises the linear slides to the specified height.
     */
    @Override
    public void run() {
        robot.linearSlide.raiseToHeight(height);
    }

    @Override
    public boolean isCommandFinished() {
        return true;
    }
}
