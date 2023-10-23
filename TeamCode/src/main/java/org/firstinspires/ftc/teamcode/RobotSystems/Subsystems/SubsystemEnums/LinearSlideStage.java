package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums;

public enum LinearSlideStage {          // Note:500 good starting distance for auto
    GROUND_STAGE (0),
    LOW_STAGE (1000),
    MID_STAGE (2000),
    HIGH_STAGE (3000);

    private final int position;
    LinearSlideStage(int position) {this.position = position; }

    public int getPosition() { return this.position; }
}
