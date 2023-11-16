package org.firstinspires.ftc.teamcode.RobotSystems.Commands;

import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.LinearSlideStage;

public class MoveLinearSlides extends Commands {
    LinearSlideStage targetStage = null;

    /**
     * Sets the target stage that the linear slides will move to when this command is run.
     *
     * @param targetStage The stage you want the linear slides to raise to.
     */
    public MoveLinearSlides(LinearSlideStage targetStage) {
        this.targetStage = targetStage;
    }

    @Override
    public void run() {
        robot.linearSlide.setStage(targetStage);
    }

    @Override
    public boolean commandFinished() {
        return true;
    }
}
