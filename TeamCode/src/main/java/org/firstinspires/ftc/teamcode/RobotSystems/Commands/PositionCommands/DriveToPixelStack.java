package org.firstinspires.ftc.teamcode.RobotSystems.Commands.PositionCommands;

import org.firstinspires.ftc.teamcode.RobotSystems.Commands.Commands;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums.PixelStack;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.FieldPosition;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

public class DriveToPixelStack extends Commands {
    // Flag to tell when the command has finished running.
    private boolean finished = false;
    private int path = 0;
    private RobotPosition targetPosition = null;

    /**
     * Determines which path the robot will take to arrive at the team's backdrop if there isn't a
     * straight path to the backdrop.
     *
     * @param path Determines what path the robot will take to reach the target pixel stack if it can't
     *             drive directly there. Below is a list explaining what path each value will cause
     *             the robot to take:
     *             <ol>
     *              <li> Drives through the stage door to get to the alliance backdrop.  </li>
     *              <li> Drives through the side door near the alliance's spike marks  </li>
     *              <li> Drives through the closest side-door to where the robot started </li>
     *             </ol>
     *             Any other value will cause the robot to drive to the desired position through the
     *             very center of the field.
     */
    public DriveToPixelStack(int path) {
        this.path = path;
        this.targetPosition = autoParams.getTargetPixelStackPosition();
    }

    /**
     * Determines which path the robot will take to arrive at the team's backdrop if there isn't a
     * straight path to the backdrop.
     *
     * @param path Determines what path the robot will take to reach the target pixel stack if it can't
     *             drive directly there. Below is a list explaining what path each value will cause
     *             the robot to take:
     *             <ol>
     *              <li> Drives through the stage door to get to the alliance backdrop.  </li>
     *              <li> Drives through the side door near the alliance's spike marks  </li>
     *              <li> Drives through the closest side-door to where the robot started </li>
     *             </ol>
     *             Any other value will cause the robot to drive to the desired position through the
     *             very center of the field.
     *             <br>
     * @param targetPixelStack The pixel stack that the robot will drive to. If no value is provided,
     *                         the the robot will just drive to the target pixel stack defined in
     *                         the autoParams provided to the command scheduler .
     */
    public DriveToPixelStack(int path, PixelStack targetPixelStack) {
        this.path = path;
        this.targetPosition = targetPixelStack.stackPosition();
    }

    @Override
    public void run() {

        // Create a reference to the robot's drivetrain to make the code look cleaner.
        DriveTrain driveTrain = robot.driveTrain;

        // Get the robot's current position.
        RobotPosition robotPosition = driveTrain.coordinateSystem.getPosition();

        // If we can't drive to the pixel stack without hitting a wall then navigate to a position
        // where we can drive to the pixel stack using the path that the programmer input in the class
        // constructor.
        if (robotPosition.x > -24) {
            navigateToPosition(driveTrain, robotPosition);
        }

        // Calculate the direction the robot has to be pointing in in order to be facing the backdrop.
        double desiredRotation = Math.toRadians(90);

        // Rotate the robot to the desiredRotation if it isn't already within 1 degree of the desired rotation.
        double distanceToRotation = driveTrain.coordinateSystem.getDistanceToRotation(desiredRotation);
        if (Math.abs(distanceToRotation) > 1) {
            driveTrain.rotateTo(desiredRotation);
        }

        // Drive the robot to the target pixel stack.
        driveTrain.driveRobotToPosition(autoParams.getTargetPixelStackPosition());

        // Tell the CommandScheduler that the command has finished running.
        finished = true;
    }

    public void navigateToPosition(DriveTrain driveTrain, RobotPosition robotPosition) {

        // Get the end location of where we want to drive to.
        RobotPosition targetPosition = calculateTargetPosition();

        /*
        If we can just drive forwards and reach a position where we can just drive and reach the
        target pixel stack, then we'll do that and skip over everything else. Additionally, the robot
        will ignore this opportunity if it would drive through the area with the spike marks unless
        specified by the programmer as to prevent accidentally underscoring a vision detection
        scored pixel. If we can't drive forwards because we will descore a pixel, then reverse so
        that we can avoid running into a pole and ruining the entire auto.
         */
        if (robotPosition.x <= 0 && !(Math.abs(robotPosition.y) > 24 && Math.abs(robotPosition.y) < 48) || path == 2) {
            driveTrain.driveRobotToPosition(new RobotPosition(-55, robotPosition.y, Math.toRadians(90)));
            return;
        } else if (Math.abs(robotPosition.y) > 24 && Math.abs(robotPosition.y) < 48) {
            driveTrain.driveRobotToPosition(new RobotPosition(24, robotPosition.y, Math.toRadians(90)));
        }

        // Drive the robot to a position where it can simply drive forwards and get to a position
        // where it can drive to the backdrop.
        driveTrain.driveRobotToPosition(new RobotPosition(24, targetPosition.y, Math.toRadians(-90)));

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
                positionOffset = new FieldPosition(-55, -12);
                break;
            case 2:
                positionOffset = new FieldPosition(-55, -36);
                break;
            case 3:
                positionOffset = new FieldPosition(-55, -60);
                break;
            default:
                positionOffset = new FieldPosition(-55, 0);
                break;
        }

        positionOffset.multiplyBy(autoParams.getDirectionalModifier());
        endingPosition.addValues(positionOffset);

        // Return the position we want the robot to drive to.
        return endingPosition;
    }

    @Override
    public boolean isCommandFinished() {
        return finished;
    }
}