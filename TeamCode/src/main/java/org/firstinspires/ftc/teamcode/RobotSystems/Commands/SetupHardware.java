package org.firstinspires.ftc.teamcode.RobotSystems.Commands;

/**
 * SetupHardware command configures the hardware and sets the robot's initial position.
 * This includes lowering the Linear Slides and positioning the robot on the field.
 */
public class SetupHardware extends Commands {

    // Flag to track whether or not the command has finished running.
    private boolean finished = false;

    /**
     * {@inheritDoc}
     * Configure / moves the hardware into a usable state. This includes lowering the Linear Slides
     * to the ground since they start in the air so that the manipulator can fit within the required
     * area. Additionally, sets the robot's position to the starting position provided through the
     * AutoParams so that all of our following commands will be accurate.
     */
    @Override
    public void run() {
        // Calculate the rotational offset the robot. (blue alliance's rotation will be reversed if
        // we don't set this to PI (180 degrees)
        double rotationalOffset = 0;
        if (autoParams.getDirectionalModifier() == -1) {
            rotationalOffset = Math.PI;
        }

        // Set the robot's position to the starting position obtained from AutoParams.
        robot.driveTrain.coordinateSystem.setRobotPosition(autoParams.getStartingPosition(), rotationalOffset);

        // Close the manipulator so that we can score the starting pixel.
        robot.manipulator.close();

        // Indicate that the command has finished running.
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