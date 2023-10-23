package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.DriveMode;

public class DriveTrain {
    private Telemetry robotTelemetry = null;
    public CoordinateSystem coordinateSystem = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DriveMode current_drive_mode = DriveMode.DEFAULT_DRIVE;
    public static final double STRAIF_OFFSET = 1.1;
    public static final double CAMERA_OFFSET = -5.9375;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Initialize hardware values
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");

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

        // Allows this drivetrain to communicate with the user.
        robotTelemetry = telemetry;

        // Tell the user that this subsystem has been successfully initialized.
        robotTelemetry.addData("->", "DriveTrain successfully initialized");
    }

    /**
     * Calculates the powers for the motors to achieve the requested
     * motion: Drive front and back, and robot rotation.
     *
     * @param driveFrontBack;      Determines how far forwards or backwards the robot has to move.
     * @param driveLeftRight;      Determines how far left or right the robot moves.
     * @param rotation;            Determines how much the robot should turn.
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
     * @param rightFrontPower;      The power that will be applied to the right front motor.
     * @param rightBackPower;       The power that will be applied to the right back motor.
     * @param leftFrontPower;       The power that will be applied to the left front  motor.
     * @param leftBackPower;        The power that will be applied to the left back  motor.
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
    }

    /**
     * Switch the robot from one mode to another, (allowing for the user to go slower or faster)
     * However, if the button pressed attempts to set the robots mode to the same mode it already
     * in, then reset the mode to default.
     *
     * @param drive_mode;       The drive mode the user wants to change the robot to.
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
