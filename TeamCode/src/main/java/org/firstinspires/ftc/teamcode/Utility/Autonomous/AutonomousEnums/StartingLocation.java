package org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums;

import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

public enum StartingLocation {
    RED_BACKDROP        (new RobotPosition(8, -64.2, Math.toRadians(0)),
                         new RobotPosition(51.6 , -36, Math.toRadians(-90)),
                         1),
    BLUE_BACKDROP       (new RobotPosition(8, 64.2, Math.toRadians(180)),
                         new RobotPosition(51.6, 36, Math.toRadians(-90)),
                        -1),
    RED_WING            (new RobotPosition(-39.8, -64.2, Math.toRadians(0)),
                         new RobotPosition(51.6 , -36, Math.toRadians(-90)),
                         1),
    BLUE_WING           (new RobotPosition(-39.8, 64.2, Math.toRadians(180)),
                         new RobotPosition(51.6, 36, Math.toRadians(-90)),
                         -1);

    private final RobotPosition startingPosition;
    private final RobotPosition backdropPosition;
    private final int directionModifier;

    /**
     * Initializes the values of each of the enum values.
     *
     * @param startingPosition The starting position of the robot on the field.
     * @param backdropPosition The position of the alliance's backdrop that the robot will score from.
     * @param directionModifier A value that can be used to invert values to account for both alliances
     *                          being at opposite ends of the field, meaning instead of running code
     *                          tailored for each separate alliance we can simply have the blue alliance
     *                          code be the red alliance code with the rotations and positions all multiple
     *                          by -1 (which inverts them).
     */
    StartingLocation(RobotPosition startingPosition, RobotPosition backdropPosition, int directionModifier) {
        this.startingPosition = startingPosition;
        this.backdropPosition = backdropPosition;
        this.directionModifier = directionModifier;
    }

    /**
     * Returns the enum's starting position.
     *
     * @return Returns the enum's starting position.
     */
    public RobotPosition startingPosition() {
        return this.startingPosition;
    }

    /**
     * Returns the enum's target backdrop position.
     *
     * @return Returns the enum's target backdrop position.
     */
    public RobotPosition backdropPosition() {
        return this.backdropPosition;
    }

    /**
     * Returns the enum's TensorFlow multiplier.
     *
     * @return Returns the enum's TensorFlow multiplier.
     */
    public int directionModifier() {
        return this.directionModifier;
    }
}

