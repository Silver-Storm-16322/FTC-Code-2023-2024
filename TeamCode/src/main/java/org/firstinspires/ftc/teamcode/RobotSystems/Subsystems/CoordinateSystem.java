package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CoordinateSystem {

    private IMU imu = null;
    private double lastRightFrontPosition = 0;
    private double lastRightBackPosition = 0;
    private double lastLeftFrontPosition = 0;
    private double lastLeftBackPosition = 0;
    private double robotRotation = 0;
    private double robotX = 0;
    private double robotY = 0;
    public static final double TICKS_PER_INCH = 57.953;

    /**
     * Initializes the imu so that we can keep track of the robot's rotation and use said rotation
     * when calculating the robot's positional change.
     *
     * @param hardwareMap Allows the robot to gain access to its hardware. (It errors if we don't do this)
     */
    public void initializeImu(HardwareMap hardwareMap) {

        // Initialize hardware variables.
        imu = hardwareMap.get(IMU.class, "imu");

        // Tell the Control hub which direction it's facing.
        IMU.Parameters imuSettings = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
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

        // Update the robot's rotation so that we can account for the robot's rotation when calculating
        // the final position.
        robotRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Recalculate the positional change to account for the robot's rotation.
        double rotatedLeftRightChange = leftRightChange * Math.cos(-robotRotation) -
                forwardsBackwardsChange * Math.sin(-robotRotation);
        double rotatedForwardsBackwardsChange = leftRightChange * Math.sin(-robotRotation) +
                forwardsBackwardsChange * Math.cos(-robotRotation);

        // Convert the above values to inches
        double xChange = rotatedLeftRightChange / TICKS_PER_INCH;
        double yChange = rotatedForwardsBackwardsChange / TICKS_PER_INCH;

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
     * Calculates the distance from the robot's current position to the provided target location.
     *
     * @param targetX The X position you want the robot move to.
     * @param targetY The Y position you want the robot to move to.
     * @param targetRotation The direction you want the robot to be facing by the end of the movement.
     * @return Returns and array containing the distance to all of the desired coordinates.
     */
    public double[] getDistanceToPosition(double targetX, double targetY, double targetRotation) {

        // Initialize return value.
        double[] distanceToTargetPosition = new double[3];

        // Update the robot's rotation so that the below calculation involving rotations are accurate.
        robotRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate how far away the robot is from the target position.
        double xDistance = targetX - robotX;
        double yDistance = targetY - robotY;
        double rotationDifference = targetRotation - robotRotation;

        // Add the above calculated values to distanceToTargetPosition[]
        distanceToTargetPosition[0] = xDistance;
        distanceToTargetPosition[1] = yDistance;
        distanceToTargetPosition[2] = robotRotation;

        // Return values to user.
        return distanceToTargetPosition;
    }

    /**
     * Returns an array containing the robot's X and Y position (In inches)
     *
     * @return Returns an array containing the robot's X and Y position (In inches) and rotation )in radians)
     */
    public double[] getPosition() {
        return new double[]{robotX, robotY, robotRotation};
    }
}
