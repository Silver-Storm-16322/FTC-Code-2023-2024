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
    public double rotationalOffset = 0;
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
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuSettings);

        // Reset the imu's angle.
        resetIMU();
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
     * Calculates the distance the robot is away from a position.
     *
     * @param targetPosition The position you want to find the distance to.
     * @return Returns the distance the robot is away from the provided target position.
     */
    public FieldPosition getDistanceToPosition(RobotPosition targetPosition) {

        // Create a new FieldPosition using the provided target position.
        // (We do this so that the TargetPosition's values don't get changed when we do the blow calculations.
        FieldPosition targetFieldPosition = new FieldPosition(targetPosition);

        // Calculate how far away the robot is from the target position.
        targetFieldPosition.subtractValues(robotPosition);

        // Return the distance (magnitude) to the target position.
        return targetFieldPosition;
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
     * Resets the IMU's rotation to help counteract IMU drift.
     */
    public void resetIMU() {
        imu.resetYaw();
    }
    /**
     * Returns the robot's position (In inches) and rotation (in radians)
     *
     * @return Returns the robot's X and Y position (In inches) and rotation )in radians)
     */
    public RobotPosition getPosition() {
        // Update the robot's rotation so that the returned value is accurate.
        updateRotation();

        // Return the robot's position.
        return robotPosition;
    }

    /**
     * Sets the robot's position to the provided RobotPosition.
     * @param position The RobotPosition you want to set the robot's position to.
     * @param rotationalOffset How much the robot is rotate by. (We set this to 180 for blue alliance
     *                         so that our robot can share the same rotations and coordinates as red
     *                         alliance.
     */
    public void setRobotPosition(RobotPosition position, double rotationalOffset) {
        this.robotPosition.x = position.x;
        this.robotPosition.y = position.y;
        this.rotationalOffset = rotationalOffset;
    }
}
