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
     * @param hardwareMap;  Allows the robot to gain access to it's hardware. (It errors if we don't do this)
     */
    public void initializeImu(HardwareMap hardwareMap) {

        // Initialize hardware variables.
        imu = hardwareMap.get(IMU.class, "imu");

        // Tell the Control hub which direction it's facing.
        IMU.Parameters imuSettings = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(imuSettings);
    }

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

        // Update the robot's rotation so that we can account for the robot's rotation when calculating
        // the final position.
        robotRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Recalculate the positional change to account for the robot's rotation.
        double rotatedLeftRightChange = leftRightChange * Math.cos(-robotRotation) -
                forwardsBackwardsChange * Math.sin(-robotRotation);
        double rotatedForwardsBackwardsChange = leftRightChange * Math.cos(-robotRotation) +
                forwardsBackwardsChange * Math.sin(-robotRotation);

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
     * Returns an array containing the robot's X and Y position (In inches)
     *
     * @return      Returns an array containing the robot's X and Y position (In inches) and rotation )in radians)
     */
    public double[] getPosition() {
        return new double[]{robotX, robotY, robotRotation};
    }

}
