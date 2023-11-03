package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.DriveMode;
import org.opencv.core.Mat;

public class DriveTrain {
    private LinearOpMode myOpMode = null;
    public CoordinateSystem coordinateSystem = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DriveMode current_drive_mode = DriveMode.DEFAULT_DRIVE;
    public static final double STRAIF_OFFSET = 1.1;

    public DriveTrain(LinearOpMode opMode) {myOpMode = opMode; }

    public void init() {

        // Initialize hardware values
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");

        // Run all motors using RUN_WITH_ENCODER so that we can use encoder related methods.
        // Commented out for bug testing purposes.
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Note: Most motors have one side reverse.
        // Due to this, we need to reverse one side of the motors so that the robot can drive straight.
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Coordinate System
        coordinateSystem = new CoordinateSystem();
        coordinateSystem.initializeImu(myOpMode.hardwareMap);

        // Tell the user that this subsystem has been successfully initialized.
        myOpMode.telemetry.addData("->", "DriveTrain successfully initialized");
    }

    /**
     * Calculates the powers for the motors to achieve the requested
     * motion: Drive front and back, and robot rotation.
     *
     * @param driveFrontBack Determines how far forwards or backwards the robot has to move.
     * @param driveLeftRight Determines how far left or right the robot moves.
     * @param rotation Determines how much the robot should turn.
     **/
    public void driveRobot(double driveFrontBack, double driveLeftRight, double rotation) {

        // Get the robots speed multiplier
        double speedMultiplier = current_drive_mode.speedMultiplier();

        // Recalculate Values
        double newDriveFrontBack = driveFrontBack * speedMultiplier;
        double newDriveLeftRight = driveLeftRight * STRAIF_OFFSET * speedMultiplier; // Multiple by STRAIF_OFFSET to counteract imperfect straifing.
        double newRotation = rotation * speedMultiplier;

        // Calculate the value that all o the values need to be divided by in order for the robot's
        // wheels to maintain a consistent ratio. This is required because motor power is capped at 1.
        double denominator =  Math.max(Math.abs(newDriveFrontBack) + Math.abs(newDriveLeftRight) + Math.abs(newRotation), 1);

        // Calculate the power levels for each motor.
        double rightFrontPower = (newDriveFrontBack - newDriveLeftRight - newRotation) / denominator;
        double rightBackPower = (newDriveFrontBack + newDriveLeftRight - newRotation)/ denominator;
        double leftFrontPower = (newDriveFrontBack + newDriveLeftRight + newRotation) / denominator;
        double leftBackPower = (newDriveFrontBack - newDriveLeftRight + newRotation) / denominator;

        // Use the pre-existing function to apply the power to the wheels.
        setDrivePower(rightFrontPower, rightBackPower, leftFrontPower, leftBackPower);
    }

    /**
     * Apply the specified power levels to their associated motors. Allows the robot to move and rotate.
     *
     * @param rightFrontPower The power that will be applied to the right front motor.
     * @param rightBackPower The power that will be applied to the right back motor.
     * @param leftFrontPower The power that will be applied to the left front  motor.
     * @param leftBackPower The power that will be applied to the left back  motor.
     */
    public void setDrivePower(double rightFrontPower, double rightBackPower, double leftFrontPower, double leftBackPower) {

        // Apply the power levels to the motors.
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);

        // Update coordinate position
        coordinateSystem.updateRobotPosition(rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

       // Tell the user what position the robot is located at.
        displayRobotPosition();
    }

    /**
     * Drives the robot to the specified position and rotation in the coordinate system.
     *
     * @param targetXPosition The target X-coordinate in inches.
     * @param targetYPosition The target Y-coordinate in inches.
     * @param targetRotation The target rotation in degrees.
     */
    public void driveRobotToPosition(double targetXPosition, double targetYPosition, double targetRotation) {

        // Convert target rotation to radians.
        double targetRotationRadians = Math.toRadians(targetRotation);

        // Get distance to target from robot's coordinate system.
        double[] targetPosition = coordinateSystem.getDistanceToPosition(targetXPosition, targetYPosition, targetRotationRadians);

        // Convert position to encoder counts.
        double targetXDistance = targetPosition[0] * CoordinateSystem.TICKS_PER_INCH;
        double targetYDistance = targetPosition[1] * CoordinateSystem.TICKS_PER_INCH;

        // Rotate the target position values so that they are independent from the rotation.
        double rotatedTargetXDistance = targetXDistance * Math.cos(-targetPosition[2]) - targetYDistance * Math.sin(-targetPosition[2]);
        double rotatedTargetYDistance = targetXDistance * Math.sin(-targetPosition[2]) + targetYDistance * Math.cos(-targetPosition[2]);

        // Move robot to the rotated position.
        setTargetPosition(rotatedTargetXDistance, rotatedTargetYDistance);
    }

    /**
     * Moves the robot to a specified target position.
     *
     * @param targetX The target X-coordinate.
     * @param targetY The target Y-coordinate.
     */
    private void setTargetPosition(double targetX, double targetY) {

        // Calculate encoder changes for each individual motor
        int rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (targetY - targetX);
        int rightBackTarget = rightBackDrive.getCurrentPosition() + (int) (targetY + targetX);
        int leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (targetY + targetX);
        int leftBackTarget = leftBackDrive.getCurrentPosition() + (int) (targetY - targetX);

        // Set target positions and motor run modes
        rightFrontDrive.setTargetPosition(rightFrontTarget);
        rightBackDrive.setTargetPosition(rightBackTarget);
        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftBackDrive.setTargetPosition(leftBackTarget);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Calculate Average target position
        double averageTargetPosition = (double)(rightFrontTarget + rightBackTarget + leftFrontTarget + leftBackTarget) / 4;
        double averageEncoderPosition = 0;

        // Set starting motor power
        double targetPower = .6;
        rightFrontDrive.setPower(targetPower);
        rightBackDrive.setPower(targetPower);
        leftFrontDrive.setPower(targetPower);
        leftBackDrive.setPower(targetPower);

        // Constantly update the user as to where the robot is on the field.
        while ((averageTargetPosition - averageEncoderPosition) / CoordinateSystem.TICKS_PER_INCH > 0.1 || rightFrontDrive.isBusy() &&
                rightBackDrive.isBusy() && leftFrontDrive.isBusy() && leftBackDrive.isBusy()) {

            // Calculate Average Encoder Position
            averageEncoderPosition = (double)(rightFrontDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition() +
                    leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition()) / 4;

            // Display robot's position
            coordinateSystem.updateRobotPosition(rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

            // Output robot's position
            displayRobotPosition();
        }

        // Update the robot's position to allow for more accurate data.
        coordinateSystem.updateRobotPosition(rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

        // Stop motors after reaching the target position
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);

        // Set motor run modes back to RUN_USING_ENCODER
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Displays what position the robot is currently located at on the field on the Telemetry.
     */
    private void displayRobotPosition() {

        // Get the robot's position
        double[] robotPosition = coordinateSystem.getPosition();

        // Convert the robot's rotation to degrees to make it easier for a human to understand.
        double robotRotationDegrees = Math.toDegrees(robotPosition[2]);

        // Tell the user their current position.
        myOpMode.telemetry.addLine("---Robot Position---");
        myOpMode.telemetry.addData("Robot X", robotPosition[0]);
        myOpMode.telemetry.addData("Robot Y", robotPosition[1]);
        myOpMode.telemetry.addData("Robot Rotation (Radians)", robotPosition[2]);
        myOpMode.telemetry.addData("Robot Rotation (Degrees)", robotRotationDegrees);
        myOpMode.telemetry.update();
    }

    /**
     * Switch the robot from one mode to another, (allowing for the user to go slower or faster)
     * However, if the button pressed attempts to set the robots mode to the same mode it already
     * in, then reset the mode to default.
     *
     * @param drive_mode The drive mode the user wants to change the robot to.
     */
    public void setDriveMode(DriveMode drive_mode) {

        // Check if drive_mode is the same as the current drive mode.
        if (drive_mode == current_drive_mode) {
            current_drive_mode = DriveMode.DEFAULT_DRIVE;
        } else {
            current_drive_mode = drive_mode;
        }
    }
}