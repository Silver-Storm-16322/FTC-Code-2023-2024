package org.firstinspires.ftc.teamcode.RobotSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class RobotHardware {

    // Declare Hardware Variables.
    private LinearOpMode myOpMode = null;
    public Vision aprilTagDetection = null;
    public LinearSlide linearSlide = null;
    public Manipulator manipulator = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DriveMode current_drive_mode = DriveMode.DEFAULT_DRIVE;

    // Autonomous coordinate variables
    private double lastRightFrontPosition = 0;
    private double lastRightBackPosition = 0;
    private double lastLeftFrontPosition = 0;
    private double lastLeftBackPosition = 0;
    private double robotX = 0;
    private double robotY = 0;

    public static final double STRAIF_OFFSET = 1.1;
    public static final double TICKS_PER_INCH = 57.953;
    public static final double CAMERA_OFFSET = -5.9375;


    public RobotHardware (LinearOpMode opMode) { this.myOpMode = opMode; }

    public void init() {

        // Initialize hardware values
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");

        // Run all motors using RUN_WITH_ENCODER so that we can use encoder related methods.
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

        // Initialize Subsystems
        linearSlide = new LinearSlide(myOpMode);
        linearSlide.init();

        manipulator = new Manipulator(myOpMode);
        manipulator.init();

        aprilTagDetection = new Vision(myOpMode);
        aprilTagDetection.init();

        // Tell the user that the hardware has been initialized.
        myOpMode.telemetry.addData("---->", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void updateRobotPosition() {

        // Calculate change in encoder positions
        double changeRightFront = rightFrontDrive.getCurrentPosition() - lastRightFrontPosition;
        double changeRightBack = rightBackDrive.getCurrentPosition() - lastRightBackPosition;
        double changeLeftFront = leftFrontDrive.getCurrentPosition() - lastLeftFrontPosition;
        double changeLeftBack = leftBackDrive.getCurrentPosition() - lastLeftBackPosition;

        // Calculate average offset
        double forwardsBackwardsChange = (changeRightFront + changeRightBack + changeLeftFront + changeLeftBack) / 4;
        double leftRightChange = ( (changeLeftFront + changeRightBack) - (changeLeftBack + changeRightFront)) / 4;

        // Convert the above values to inches
        double yChange = forwardsBackwardsChange / TICKS_PER_INCH;
        double xChange = leftRightChange / TICKS_PER_INCH;

        // Update Position
        robotY = robotY + yChange;
        robotX = robotX + xChange;
        lastRightFrontPosition = rightFrontDrive.getCurrentPosition();
        lastRightBackPosition = rightBackDrive.getCurrentPosition();
        lastLeftFrontPosition = leftFrontDrive.getCurrentPosition();
        lastLeftBackPosition = leftBackDrive.getCurrentPosition();

        // Tell the user what the new positions are
        myOpMode.telemetry.addLine("------ Robot Position Data ------");
        myOpMode.telemetry.addData("Robot X", robotX +" in");
        myOpMode.telemetry.addData("Robot Y", robotY + "in");
        myOpMode.telemetry.update();
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

        // Update Position
        updateRobotPosition();
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
