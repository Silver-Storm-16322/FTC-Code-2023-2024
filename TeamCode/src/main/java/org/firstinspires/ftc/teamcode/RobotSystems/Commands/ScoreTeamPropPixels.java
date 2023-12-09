package org.firstinspires.ftc.teamcode.RobotSystems.Commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums.TeamPropLocation;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

public class ScoreTeamPropPixels extends Commands {

    // Flag to tell when the command has finished running.
    private boolean finished = false;
    private int parkingLocation = 0;
    private int path = 0;
    private LinearOpMode myOpMode = null;
    private TeamPropLocation teamPropLocation = null;
    private RobotPosition startingPosition = null;
    private boolean scorePurplePixelFirst = false;

    /**
     *
     * @param scorePurplePixelFirst Determines whether or not the robot will score the purple pixel
     *                              on the spike mark first. If this value is false the yellow pixel
     *                              will be scored on the backdrop first.
     */
    public ScoreTeamPropPixels(boolean scorePurplePixelFirst, int path,LinearOpMode opMode) {
        this.scorePurplePixelFirst = scorePurplePixelFirst;
        this.myOpMode = opMode;
        this.path = path;
    }

    @Override
    public void run() {

        // Get the location of the team prop
        teamPropLocation = getTeamProp();

        // Set the starting position.
        startingPosition = autoParams.getStartingPosition();

        // Score Purple Pixel
        scorePurplePixel();

    }

    /**
     * Finds the location of the team prop on the field.
     *
     * @return Returns the position on the field where the team prop is located.
     */
    private TeamPropLocation getTeamProp() {

        // Get the team prop
        Recognition teamProp = robot.vision.getDetectedTeamProps(autoParams.getDirectionalModifier());
        TeamPropLocation teamPropLocation;

        // If we can see the team prop then check determine where it is located. Otherwise, set the prop
        // location to left or right depending on which alliance we are on.
        if (teamProp != null) {

            // Get the position on the camera where the left side of the team prop is located.
            double leftCoordinate = teamProp.getLeft();

            /*
            Check if the team prop is located on the center spike mark. We do this by checking if the
            prop is on the left side of the screen (Since the center will be on the left side of the
            screen for both alliances). Otherwise, set the team prop to left or right depending on
            the alliance. (As mentioned above this is because the alliances are both mirrored.
             */
            if (leftCoordinate <= 120 ) {
                teamPropLocation = TeamPropLocation.CENTER;
            } else {
                teamPropLocation = TeamPropLocation.RIGHT;
            }
        } else {

            /*
            Set the team prop location to left if the robot is on red alliance and right if the robot
            is on blue alliance. This is done because it is the only area our camera can't see and the
            value is different for blue alliance because it is rotated 180 degrees from red alliance.
             */
            teamPropLocation = TeamPropLocation.LEFT;
        }

        // Return the location of the team prop.
        return teamPropLocation;
    }

    /**
     * This method scores the purple pixel on the specified spike mark (as labeled by the team prop)
     */
    private void scorePurplePixel() {

        /*
        Create a robot position using the robot's starting position. This is done so that we can offset
        where the robot has to drive to based on the detected team prop as well as our starting location.
         */
        RobotPosition scoringPosition = new RobotPosition(startingPosition);

        // Determine which route to take to reach the target spike mark.
        switch (teamPropLocation) {
            case CENTER:

                // Simply drive straight forwards.
                scoringPosition.y += 26;
                break;
            case RIGHT:

                // Check whether or not we are able to get away with just driving to the spike mark.
                // If we cat simply drive there, then calculate the optimal path to take to get there.
                if (startingPosition.x > 0) {
                    scoringPosition.y += 25;
                    scoringPosition.x += 11.8;
                } else {
                    scoringPosition.y += 29.9;
                    scoringPosition.x += 2;
                    scoringPosition.rotation = Math.toRadians(-90);
                }
                break;
            case LEFT:

                // Check whether or not we are able to get away with just driving to the spike mark.
                // If we cat simply drive there, then calculate the optimal path to take to get there.
                if (startingPosition.x < 0) {
                    scoringPosition.y += 25;
                    scoringPosition.x -= 11.8;
                } else {
                    scoringPosition.y += 29.9;
                    scoringPosition.x -= 2;
                    scoringPosition.rotation = Math.toRadians(90);
                }
                break;
        }

        /*
        Check if the robot is able to drive in a straight line. This is done by checking whether
        or not the robot has to rotate to reach it's target (which will only happen if we can't
        drive straight towards it). The reason we check for this is to make driving in a straight
        line more efficient since it only needs to drive once.
         */
        if (scoringPosition.rotation != 0) {
            robot.driveTrain.driveRobotToPosition(new RobotPosition(startingPosition.x, scoringPosition.y, 0));
        }

        // If the robot isn't facing within 1 degree of it's target rotation then rotate it so that
        // it is facing the correct direction.
        if (robot.driveTrain.coordinateSystem.getDistanceToRotation(scoringPosition.rotation) > Math.toRadians(1)) {
            robot.driveTrain.driveRobotToPosition(new RobotPosition(startingPosition.x + (Math.signum(startingPosition.x)) * 3, scoringPosition.y, scoringPosition.rotation));
        }

        // Drive the robot to it's end destination. This is where we are able to score the pixel on
        // the spike mark and score 20 points)
        robot.driveTrain.driveRobotToPosition(new RobotPosition(scoringPosition.x, scoringPosition.y, scoringPosition.rotation));

        /*
        Drop the pixel on the spike mark. Then, raise the linear slides so that we don't have to deal
        with any unintentional collisions that may occur when trying to get the yellow pixel. We then
        wait a bit for the linear slides to raise before
         */
        robot.manipulator.open();
        myOpMode.sleep(1500);

        // Determine where the robot has to drive to in order to be able to grab the pixel.
        RobotPosition yellowPixelGrabbingLocation = new RobotPosition(startingPosition.x +
                (Math.signum(startingPosition.x) * 4.5), startingPosition.y + 2,
                Math.toRadians(90 * Math.signum(startingPosition.x)));

        // Navigate to the pixel so that we don't collide with part of the field.
        if (Math.abs(scoringPosition.rotation) > Math.toRadians(1)) {

            // Drive back a little bit so that we can rotate without hitting parts of the field.
            robot.driveTrain.driveRobotToPosition(new RobotPosition(yellowPixelGrabbingLocation.x,
                    scoringPosition.y,
                    scoringPosition.rotation));

            // Rotate the robot so that we are facing the direction of the pixel we want to grab.
            robot.driveTrain.driveRobotToPosition(new RobotPosition(yellowPixelGrabbingLocation.x,
                    scoringPosition.y,
                    yellowPixelGrabbingLocation.rotation));
        }

        // Drive the robot to the position required for it to grab the yellow pixel.
        robot.driveTrain.driveRobotToPosition(yellowPixelGrabbingLocation);

        // Drive forwards in the direction of the pixel so that we can mitigate the effects of slippage
        // and ensure that we are able to grab the pixel.
        robot.driveTrain.driveRobotToPosition(new RobotPosition(yellowPixelGrabbingLocation.x +
                (Math.signum(startingPosition.x) * -6),yellowPixelGrabbingLocation.y,
                yellowPixelGrabbingLocation.rotation));

        // Grab the pixel/
        robot.manipulator.close();
        myOpMode.sleep(1000);

        // Drive backwards a bit so that we are able to make the required movements to reach the desired
        // parking location.
        robot.driveTrain.driveRobotToPosition(new RobotPosition(yellowPixelGrabbingLocation.x +
                (Math.signum(startingPosition.x) * 6),yellowPixelGrabbingLocation.y,
                yellowPixelGrabbingLocation.rotation));


        // Park the robot in the desired location.
        driveToParkingLocation();
    }

    /**
     * This method makes the robot drive to the desired parking location and drop it's currently held
     * pixel.
     */
    private void driveToParkingLocation() {

        // Get the robot's current position on the field. This will help us determine how we try to
        // reach the desired parking location.
        RobotPosition robotPosition = robot.driveTrain.coordinateSystem.getPosition();

        // Calculate where we have to drive to in order to reach desired parking location.
        RobotPosition backdropPosition = autoParams.getBackdropPosition();
        RobotPosition parkingPosition;

        // Determine which side of the backdrop we want to park in. This is determined by the
        // paringLocation value set by the user when the OpMode is started.
        if (parkingLocation == 0) {

            // Set parking location to the parking location on the right side of the backdrop.
            parkingPosition = new RobotPosition(55.9 * Math.signum(backdropPosition.x),
                    0, backdropPosition.rotation);
        } else {

            // Set parking location to the parking location on the left side of the backdrop.
            parkingPosition = new RobotPosition(55.9 * Math.signum(backdropPosition.x),
                    53.7, backdropPosition.rotation);
        }

        /*
        Check if the robot is on the same side of teh field as the backdrop. If we are, then just
        drive to the desired parking location. Otherwise, navigate to a position where we can drive
        to the parking location.
         */
        if (Math.signum(robotPosition.x) != Math.signum(parkingPosition.x)) {

            // Drive to the wing. This allows us to navigate the the parking location while avoiding
            // collisions with game elements and other team's robot.
            robot.driveTrain.driveRobotToPosition(new RobotPosition(startingPosition.x +
                    (Math.signum(startingPosition.x) * 24),
                    startingPosition.y + 2,
                    Math.toRadians(0)));

            // Then, drive forwards so that we are able to drive directly to the parking location.
            robot.driveTrain.driveRobotToPosition(new RobotPosition(startingPosition.x +
                    (Math.signum(startingPosition.x) * 24),
                    backdropPosition.y,
                    Math.toRadians(0)));
        }

        // Drive to the parking position.
        robot.driveTrain.driveRobotToPosition(new RobotPosition(parkingPosition.x / 2, parkingPosition.y / 2, parkingPosition.rotation));
        robot.driveTrain.driveRobotToPosition(parkingPosition);

        // Drop the pixel so that we are able to score 3 points.
        robot.manipulator.open();
    }

    @Override
    public boolean isCommandFinished() {
        return finished;
    }
}