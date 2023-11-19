package org.firstinspires.ftc.teamcode.RobotSystems.Commands.PositionCommands;

import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.FieldPosition;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

/**
 * The DriveToTeamBackdrop command will drive the robot to the center of the team's backdrop
 * regardless of where the robot is currently located on the field.
 */
public class DriveToTeamBackdrop extends Commands {

    // Flag to track whether or not the command has finished running.
    private boolean finished = false;
    private int path;

    /**
     * Determines which path the robot will take to arrive at the team's backdrop if there isn't a
     * straight path to the backdrop.
     *
     * @param path Determines what pth the robot will take to reach the team backdrop if it can't
     *             simply drive directly to the team backdrop. Below is a list explaining what path
     *             each value will cause the robot to take:
     *             <ol>
     *              <li> Drives through the stage door to get to the alliance backdrop.  </li>
     *              <li> Drives through the side door near the alliance's spike marks  </li>
     *              <li> Drives through the closest side-door to where the robot started </li>
     *             </ol>
     *             Any other value will cause the robot to drive to the desired position through the
     *             very center of the field.
     */
    public DriveToTeamBackdrop(int path) {
        this.path = path;
    }

    /**
     * {@inheritDoc}
     * Drives the robot to the alliance's backdrop and will navigate to a position where the robot
     * can drive to the alliance's backdrop if required.
     */
    @Override
    public void run() {

        // Get the robot's position so that we can move the robot to the backdrop no matter where
        // the robot is located on the field.
        DriveTrain driveTrain = robot.driveTrain;
        RobotPosition robotPosition = driveTrain.coordinateSystem.getPosition();

        /*
        If the robot doesn't have a clear shot towards the backdrop, then drive to a position derived
        from the path number specified above where the robot can drive to the backdrop.
         */
        if (robotPosition.x < 24) {
            navigateToPosition(driveTrain, robotPosition);
        }

        // Calculate the direction the robot has to be pointing in in order to be facing the backdrop.
        double desiredRotation = Math.toRadians(-90);

        // Rotate the robot to the desiredRotation if it isn't already within 1 degree of the desired rotation.
        double distanceToRotation = driveTrain.coordinateSystem.getDistanceToRotation(desiredRotation);
        if (Math.abs(distanceToRotation) > 1) {
            driveTrain.rotateTo(desiredRotation);
        }

        // Drive to the backdrop.
        driveTrain.driveRobotToPosition(autoParams.getBackdropPosition());

        // Tell the CommandScheduler that the command finished running.
        finished = true;
    }

    /**
     * Navigate to a position on the field where the robot is able to drive to the alliance backdrop.
     *
     * @param driveTrain The robot's drive train.
     * @param robotPosition The position that the robot started at when the command began.
     */
    private void navigateToPosition(DriveTrain driveTrain, RobotPosition robotPosition) {

        // Get the end location of where we want to drive to.
        RobotPosition targetPosition = calculateTargetPosition();

        /*
        If we can just drive forwards and reach a position where we can just drive and reach the
        alliance backdrop, then we'll do that and skip over everything else. Additionally, the robot
        will ignore this opportunity if it would drive through the area with the spike marks unless
        specified by the programmer as to prevent accidentally underscoring a vision detection
        scored pixel. If we can't drive forwards because we will descore a pixel, then reverse so
        that we can avoid running into a pole and ruining the entire auto.
         */
        if (robotPosition.x >= -24 && !(Math.abs(robotPosition.y) > 24 && Math.abs(robotPosition.y) < 48) || path == 2) {
            driveTrain.driveRobotToPosition(new RobotPosition(24, robotPosition.y, Math.toRadians(-90)));
            return;
        } else if (Math.abs(robotPosition.y) > 24 && Math.abs(robotPosition.y) < 48) {
            driveTrain.driveRobotToPosition(new RobotPosition(-48, robotPosition.y, Math.toRadians(-90)));
        }

        // Drive the robot to a position where it can simply drive forwards and get to a position
        // where it can drive to the backdrop.
        driveTrain.driveRobotToPosition(new RobotPosition(-48, targetPosition.y, Math.toRadians(-90)));

        // Drive the robot to the calculated ending position.
        driveTrain.driveRobotToPosition(targetPosition);
    }

    /**
     * This method returns the position the programmer wants the robot to navigate to based on what
     * alliance the robot is currently on.
     *
     * @return The position the robot will navigate to.
     */
    private RobotPosition calculateTargetPosition() {

        // Create objects //
        RobotPosition endingPosition = new RobotPosition(0, 0, Math.toRadians(-90));
        FieldPosition positionOffset;

        // Calculate how far from the origin the robot has to drive in order to reach each gate.
        // We calculate a position offset so that we can use the built-in methods to easily multiply
        // the offset by the directional offset and have it work across both alliances.
        switch (path) {
            case 1:
                positionOffset = new FieldPosition(24, -12);
                break;
            case 2:
                positionOffset = new FieldPosition(24, -36);
                break;
            case 3:
                positionOffset = new FieldPosition(24, -60);
                break;
            default:
                positionOffset = new FieldPosition(24, 0);
                break;
        }
        positionOffset.multiplyBy(autoParams.getDirectionalModifier());
        endingPosition.addValues(positionOffset);

        // Return the position we want the robot to drive to.
        return endingPosition;
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