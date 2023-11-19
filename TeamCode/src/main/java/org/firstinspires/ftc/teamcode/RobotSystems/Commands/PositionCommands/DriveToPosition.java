package org.firstinspires.ftc.teamcode.RobotSystems.Commands.PositionCommands;

import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

/**
 * This command drives the robot to the provided position on the field and will will not avoid
 * obstacles on it's way to the provided position.
 */
public class DriveToPosition extends Commands {
    // Flag to tell when the command has finished running.
    private boolean finished = false;
    private RobotPosition targetPosition;

    /**
     * Sets what position the robot will drive to when the command is run.
     *
     * @param x The X coordinate of the position you want the robot to drive to (in inches)
     * @param y The Y coordinate of the position you want the robot to drive to (in inches)
     * @param rotation What direction you want the robot to be facing after it moves
     */
    public DriveToPosition(double x, double y, int rotation) {

        // Convert the input rotation (in degrees) to radians.
        double rotationInRadians = Math.toRadians(rotation);

        // Create a RobotPosition object with the provided data.
        targetPosition = new RobotPosition(x, y, rotationInRadians);
    }

    /**
     * {@inheritDoc}
     * Drives the robot to the provided position. Will not avoid obstacles while driving to the input
     * position.
     */
    @Override
    public void run() {

        // Drive the robot to the targetPosition
        robot.driveTrain.driveRobotToPosition(targetPosition);

        // Tell the CommandScheduler that the command has finished running
        finished = true;
    }

    /**
     * {@inheritDoc}
     * Returns true if the command finished running.
     *
     * @return Whether or not the command has finished running. Otherwise, false.
     */
    @Override
    public boolean isCommandFinished() {
        return finished;
    }
}
