package org.firstinspires.ftc.teamcode.RobotSystems.Commands.ManipulatorAndSlideCommands;

import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.LinearSlideStage;

public class MoveLinearSlides extends Commands {
    LinearSlideStage targetStage = null;

    /**
     * Sets what stage the command will move the LinearSlides to.
     *
     * @param targetStage The stage you want the LinearSlides to move to.
     */
    public MoveLinearSlides(LinearSlideStage targetStage) {
        this.targetStage = targetStage;
    }

    @Override
    public void run() {
        robot.linearSlide.setStage(targetStage);
    }

    @Override
    public boolean isCommandFinished() {
        return true;
    }
}
