package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

public class CoordinateSystem {

    private double lastRightFrontPosition = 0;
    private double lastRightBackPosition = 0;
    private double lastLeftFrontPosition = 0;
    private double lastLeftBackPosition = 0;
    private double robotX = 0;
    private double robotY = 0;
    public static final double TICKS_PER_INCH = 57.953;

    /**
     * Updates the robot's (X, Y) position based on the change in encoder counts across all motors.
     *
     * @param currentRightFrontPosition;        The current position (encoder counts) of the RightFrontMotor
     * @param currentRightBackPosition;         The current position (encoder counts) of the RightBackMotor
     * @param currentLeftFrontPosition;         The current position (encoder counts) of the LeftFrontMotor
     * @param currentLeftBackPosition;          The current position (encoder counts) of the LeftBackMotor
     */
    public void updateRobotPosition(double currentRightFrontPosition, double currentRightBackPosition,
                                    double currentLeftFrontPosition, double currentLeftBackPosition) {

        // Calculate change in encoder positions
        double changeRightFront = currentRightFrontPosition - lastRightFrontPosition;
        double changeRightBack = currentRightBackPosition - lastRightBackPosition;
        double changeLeftFront = currentLeftFrontPosition - lastLeftFrontPosition;
        double changeLeftBack = currentLeftBackPosition - lastLeftBackPosition;

        // Calculate average offset
        double leftRightChange = ( (changeLeftFront + changeRightBack) - (changeLeftBack + changeRightFront)) / 4;
        double forwardsBackwardsChange = (changeRightFront + changeRightBack + changeLeftFront + changeLeftBack) / 4;

        // Convert the above values to inches
        double xChange = leftRightChange / TICKS_PER_INCH;
        double yChange = forwardsBackwardsChange / TICKS_PER_INCH;

        // Update robot position
        robotX += xChange;
        robotY += yChange;

        // Update last position
        lastRightFrontPosition = currentRightFrontPosition;
        lastRightBackPosition = currentRightBackPosition;
        lastLeftFrontPosition = currentLeftFrontPosition;
        lastLeftBackPosition = currentLeftBackPosition;
    }

    /**
     * Returns an array containing the robot's X and Y position (In inches)
     *
     * @return      Returns an array containing the robot's X and Y position (In inches)
     */
    public double[] getPosition() {
        return new double[]{robotX, robotY};
    }

}
