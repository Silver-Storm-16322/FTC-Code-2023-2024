package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums.PixelStack;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums.StartingLocation;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

public class AutoParams {
    private StartingLocation startingLocation;
    private PixelStack targetPixelStack;

    /**
     * Initializes AutoParams with specified values.
     *
     * @param startingLocation The starting position of the robot. Choose from a StartingLocation,
     *                         representing predefined positions.
     * @param targetPixelStack The stack of pixels that the robot will primarily go to to grab pixels
     *                         to score on the backdrop. PixelStack1 is the closest stack of pixels
     *                         to the red alliance and PixelStack5 is the closest stack of pixels to
     *                         the blue alliance.
     */
    public AutoParams(StartingLocation startingLocation, PixelStack targetPixelStack) {

        // Make sure that the programmer input correct data.
        if (startingLocation == null || targetPixelStack == null) {
            throw new IllegalArgumentException("Starting position and target position cannot be null..");
        }

        // Set this AutoParams' data to the provided values.
        this.startingLocation = startingLocation;
        this.targetPixelStack = targetPixelStack;
    }

    /**
     * Sets this AutoParam's starting location to the provided value.
     *
     * @param startingLocation the starting location you want to change this AutoParam's starting location
     *                         to.
     */
    public void setStartingLocation(StartingLocation startingLocation) {
        this.startingLocation = startingLocation;
    }

    /**
     * Sets this AutoParam's target pixel stack.
     *
     * @param targetPixelStack the pixel stack you want the robot to score from during auto.
     */
    public void setTargetPixelStack(PixelStack targetPixelStack) {
        this.targetPixelStack = targetPixelStack;
    }

    /**
     * Returns this AutoParam's starting location.
     *
     * @return Returns this AutoParam's starting location.
     */
    public StartingLocation getStartingLocation() {
        return startingLocation;
    }

    /**
     * Returns this AutoParam's target pixel stack.
     *
     * @return Returns this AutoParam's target pixel stack.
     */
    public PixelStack getTargetPixelStack() {
        return targetPixelStack;
    }
    /**
     * Gets the starting position of the robot.
     *
     * @return The starting position as a RobotPosition.
     */
    public RobotPosition getStartingPosition() {
        return this.startingLocation.startingPosition();
    }

    /**
     * Gets the position of the alliance's backdrop.
     *
     * @return The position of the alliance's backdrop as a RobotPosition.
     */
    public RobotPosition getBackdropPosition() {
        return this.startingLocation.backdropPosition();
    }

    /**
     * Gets the amount that we have to multiply in order to invert values when running blue alliance
     * autonomous programs so that we can reduce the amount of code we write.
     *
     * @return The value we have to multiply values by in order to account for the differences between
     *         alliances.
     */
    public int getDirectionalModifier() {
        return this.startingLocation.directionModifier();
    }

    /**
     * Gets the position of the pixel stack we want to grab and score pixels from.
     *
     * @return The position of the targetPixelStack as a RobotPosition.
     */
    public RobotPosition getTargetPixelStackPosition() {
        return this.targetPixelStack.stackPosition();
    }
}