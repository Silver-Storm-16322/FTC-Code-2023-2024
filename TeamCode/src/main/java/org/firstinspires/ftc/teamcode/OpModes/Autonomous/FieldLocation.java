package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

public enum FieldLocation {
    RED_BACKDROP        (new RobotPosition(0, 0, 0),
                         new RobotPosition(0, 0, 0)),
    RED_WING            (new RobotPosition(0, 0, 0),
                         new RobotPosition(0, 0, 0)),
    BLUE_BACKDROP       (new RobotPosition(0, 0, 0),
                         new RobotPosition(0, 0, 0)),
    BLUE_WING           (new RobotPosition(0, 0, 0),
                         new RobotPosition(0, 0, 0));

    private final RobotPosition startingPosition;
    private final RobotPosition backdropPosition;
    FieldLocation(RobotPosition startingPosition, RobotPosition backdropPosition) {
        this.startingPosition = startingPosition;
        this.backdropPosition = backdropPosition;
    }

    public RobotPosition getStartingPosition() {return this.startingPosition;}
    public RobotPosition getBackdropPosition() {return this.backdropPosition;}
}
