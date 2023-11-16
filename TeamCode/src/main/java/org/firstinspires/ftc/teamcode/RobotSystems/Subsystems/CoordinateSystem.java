package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.FieldPosition;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

public class CoordinateSystem {

    private IMU imu = null;
    private double lastRightFrontPosition = 0;
    private double lastRightBackPosition = 0;
    private double lastLeftFrontPosition = 0;
    private double lastLeftBackPosition = 0;
    private RobotPosition robotPosition = new RobotPosition(0, 0, 0);
    public static final double TICKS_PER_INCH = 57.953;

    /**
     * Initializes the imu so that we can keep track of the robot's rotation and use said rotation
     * when calculating the robot's positional change.
     *
     * @param hardwareMap Allows the robot to gain access to its hardware. (It errors if we don't do this)
     */
    public void initializeImu(HardwareMap hardwareMap) {

        // Create the IMU parameters that tell the IMU what direction its facing and allow us to use the
        // robot's rotation for various calculations.
        IMU.Parameters imuSettings = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuSettings);

        // reset the imu's angle.
        imu.resetYaw();
    }

    /**
     * Updates the robot's (X, Y) position based on the change in encoder counts across all motors.
     *
     * @param currentRightFrontPosition The current position (encoder counts) of the RightFrontMotor
     * @param currentRightBackPosition The current position (encoder counts) of the RightBackMotor
     * @param currentLeftFrontPosition The current position (encoder counts) of the LeftFrontMotor
     * @param currentLeftBackPosition The current position (encoder counts) of the LeftBackMotor
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

        // Create a Vector2 storing the average positional change in the robot.
        FieldPosition positionChange = new FieldPosition(leftRightChange, forwardsBackwardsChange);

        // Update the robot's rotation so that we can account for the robot's rotation when calculating
        // the final position.
        updateRotation();

        // Recalculate the positional change to account for the robot's rotation.
        // The rotation value is negative so that we can untangle the rotation from the
        // encoder counts, since they were already rotated.
        positionChange.rotateVector(-robotPosition.rotation);

        // Convert the above values to inches
        positionChange.divideBy(TICKS_PER_INCH);

        // Update robot position
        robotPosition.addValues(positionChange);

        // Update last position
        lastRightFrontPosition = currentRightFrontPosition;
        lastRightBackPosition = currentRightBackPosition;
        lastLeftFrontPosition = currentLeftFrontPosition;
        lastLeftBackPosition = currentLeftBackPosition;
    }

    /**
     * Calculates the distance from the robot's current position to the provided target location.
     *
     * @param targetPosition The position you want to move the robot to.
     * @return Returns a Vector3 containing the distance to the
     */
    public RobotPosition getDistanceToPosition(RobotPosition targetPosition) {

        // Update the robot's rotation so that the below calculation involving rotations are accurate.
        updateRotation();

        // Calculate how far away the robot is from the target position.
        targetPosition.subtractValues(new FieldPosition(robotPosition.x, robotPosition.y));

        // Rotate the Target Position to account for the robot's rotation.
        targetPosition.rotateVector(robotPosition.rotation);

        // Return values to user.
        return targetPosition;
    }

    /**
     * Given an angle (in radians), calculate the minimum amount the robot has to rotate in order to be facing in
     * the provided angle's direction.
     *
     * @param targetRotation An angle in radians.
     * @return The minimum amount the robot has to rotate by (in radians) in order to be facing in the direction
     *         of the targetRotation.
     */
    public double getDistanceToRotation(double targetRotation) {

        // Update the robot's rotation so that the below calculations are accurate.
        updateRotation();

        // Calculate the difference between the angle we want to rotate to and the direction the robot is facing.
        double rotationalDifference = targetRotation - robotPosition.rotation;

        // We use while-loops instead of if-statements in case the angle is greater than 360 degrees
        // so that we are still able to calculate the minimum rotation the robot has to make in order to
        // be facing the provided angle.
        while (rotationalDifference > Math.PI) {
            rotationalDifference -= 2 * Math.PI;
        }
        while (rotationalDifference < -Math.PI) {
            rotationalDifference += 2 * Math.PI;
        }

        // Return the minimum angle (in radians) the robot has to rotate by in order to be facing the provided angle.
        return rotationalDifference;
    }

    /**
     * Updates the robot's rotation.
     */
    public void updateRotation() {
        robotPosition.rotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Returns the robot's position (In inches) and rotation (in radians)
     *
     * @return Returns the robot's X and Y position (In inches) and rotation )in radians)
     */
    public RobotPosition getPosition() {
        return robotPosition;
    }
}
