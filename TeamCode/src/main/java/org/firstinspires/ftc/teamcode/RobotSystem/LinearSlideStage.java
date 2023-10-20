package org.firstinspires.ftc.teamcode.RobotSystem;

public enum LinearSlideStage {
    GROUND_STAGE (0),
    LOW_STAGE (288),
    MID_STAGE (576),
    HIGH_STAGE (1152);

    private final int position;
    LinearSlideStage(int position) {this.position = position; }

    int getPosition() { return this.position; }
}
