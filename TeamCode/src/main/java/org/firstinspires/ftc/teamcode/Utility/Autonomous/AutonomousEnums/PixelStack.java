package org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums;

import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

// Enum value names were Paul's idea! :D
public enum PixelStack {
    RED_SIDE_STACK_1    (new RobotPosition(-60.3, -36.1, Math.toRadians(90))),
    RED_SIDE_STACK_2    (new RobotPosition(-60.3, -23.9, Math.toRadians(90))),
    RED_SIDE_STACK_3    (new RobotPosition(-60.3, -11.8, Math.toRadians(90))),
    BLUE_SIDE_STACK_3   (new RobotPosition(-60.3, 11.6, Math.toRadians(90))),
    BLUE_SIDE_STACK_2   (new RobotPosition(-60.3, 23.9, Math.toRadians(90))),
    BLUE_SIDE_STACK_1   (new RobotPosition(-60.3, 36.1, Math.toRadians(90)));

    private final RobotPosition stackPosition;
    PixelStack(RobotPosition stackPosition) {
        this.stackPosition = stackPosition;
    }

    /**
     * Returns the position the robot needs to drive to on the field in order to pick up pixels from
     * the desired pixel stack.
     *
     * @return Returns this enum's stackPosition value.
     */
    public RobotPosition stackPosition() {
        return this.stackPosition;
    }
}
