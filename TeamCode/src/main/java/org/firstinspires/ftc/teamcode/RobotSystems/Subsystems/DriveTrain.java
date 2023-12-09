package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.DriveMode;
import org.firstinspires.ftc.teamcode.Utility.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utility.FileEx;
import org.firstinspires.ftc.teamcode.Utility.InputController;
import org.firstinspires.ftc.teamcode.Utility.Controllers.SingleMotorPIDController;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.FieldPosition;
import org.firstinspires.ftc.teamcode.Utility.Controllers.TurnToPIDController;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

public class DriveTrain {
    private LinearOpMode myOpMode = null;
    public CoordinateSystem coordinateSystem = null;
    public PIDController pidController = null; // Public so that it can be tuned within an OpMode.
    private FileEx pidCoefficients = new FileEx("PIDCoefficients.txt");
    private DriveMode current_drive_mode = DriveMode.DEFAULT_DRIVE;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private boolean fieldCentric = false;
    public static final double STRAFE_OFFSET = 1.1;
    public static final double MAX_AUTONOMOUS_SPEED = .7;
    public DriveTrain(LinearOpMode opMode) {myOpMode = opMode; }

    /**
     * Initializes all of the driveTrain's hardware.
     */
    public void init() {

        // Initialize hardware values
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "right_back_drive");
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "left_back_drive");

        // Set the ZeroPowerBehaviour to BRAKE so that the robot stops faster.
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run all motors using RUN_WITH_ENCODER so that we can use encoder related methods.
        // Commented out for bug testing purposes.
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Note: Most motors have one side reverse.
        // Due to this, we need to reverse one side of the motors so that the robot can drive straight.
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Coordinate System
        coordinateSystem = new CoordinateSystem();
        coordinateSystem.initializeImu(myOpMode.hardwareMap);

        // Initialize PID Controller
        pidController = new PIDController(0, 0, 0);

        // Set all of the PID coefficients to the values stored in the file.
        pidController.setKp(pidCoefficients.getValue("kp", Double.class));
        pidController.setKi(pidCoefficients.getValue("ki", Double.class));
        pidController.setKd(pidCoefficients.getValue("kd", Double.class));

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

        // Declare variables so that their value can be calculated differently based on whether or not
        // field centric is enabled.
        double newDriveFrontBack;
        double newDriveLeftRight;
        double newRotation;

        /*
        If field centric is enabled then rotate the provided values so that the robot can drive
        independent form it's rotation. Otherwise, just set the values to whatever was provided by
        the user.
         */
        if (fieldCentric) {
            // Get the robot's rotation so the below calculations will be accurate.
            double robotRotation = coordinateSystem.getPosition().rotation;

            // Rotate the values so that the robot can drive in a field centric way.
            newDriveLeftRight = driveLeftRight * Math.cos(-robotRotation) - driveFrontBack * Math.sin(-robotRotation);
            newDriveFrontBack = driveLeftRight * Math.sin(-robotRotation) + driveFrontBack * Math.cos(-robotRotation);
        } else {
            newDriveLeftRight = driveLeftRight;
            newDriveFrontBack = driveFrontBack;
        }
        newRotation = rotation;

        // Get the robots speed multiplier
        double speedMultiplier = current_drive_mode.speedMultiplier();

        // Recalculate the values to adjust for the speed multiplier and strafe offset.
        newDriveFrontBack *= speedMultiplier;
        newDriveLeftRight *= STRAFE_OFFSET * speedMultiplier;
        newRotation *= speedMultiplier;

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
        coordinateSystem.updateRobotPosition(
                rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

       // Tell the user what position the robot is located at.
        displayRobotPosition();
    }

    /**
     * Moves the robot a specified amount of inches in any direction relative to the robot.
     *
     * @param targetPosition The position you want the robot to move to.
     */
    public void driveRobotToPosition(RobotPosition targetPosition) {

        // Input controller //
        InputController inputController = new InputController(0.2, 1);

        // Create a PID controller so that we can smoothly rotate the robot and maintain a constant rotation //
        TurnToPIDController rotationController = new TurnToPIDController(0.02, 0, 0.002);

        // Create PID Controllers to smoothly move the robot to the provided target position.
        PIDController xController = new PIDController(pidController);
        PIDController yController = new PIDController(pidController);

        // Keep track of how close the robot is to the target position
        // (We use 9999 as a placeholder because we don't want to prematurely stop the loop)
        double rotationalError = 9999;
        double xDistance = 9999;
        double yDistance = 9999;

        // Keep moving until each motor is within 6 encoder ticks of their desired position.
        while (myOpMode.opModeIsActive() && ((xDistance > .1 || yDistance > .1) ||
                Math.abs(rotationalError) > Math.toRadians(1))) {

            // Update how far we are from the target position
            FieldPosition distanceVector = coordinateSystem.getDistanceToPosition(targetPosition);
            xDistance = Math.abs(distanceVector.x);
            yDistance = Math.abs(distanceVector.y);

            // Update the rotational difference
            rotationalError = coordinateSystem.getDistanceToRotation(targetPosition.rotation);

            // Calculate what % of the maximum autonomous speed the robot should be moving.
            // (Increases over time)
            double smoothingFactor = inputController.smoothInput(MAX_AUTONOMOUS_SPEED);

            // Get the robot's position so that we are able to have accurate data for the below calculations.
            RobotPosition robotPosition = coordinateSystem.getPosition();

            // Calculate the directional powers required to drive in the desired direction.
            double xPower = xController.update(targetPosition.x, robotPosition.x);
            double yPower = yController.update(targetPosition.y, robotPosition.y);

            // Rotate the values to account for the robot's rotation.
            double rotatedXPower = xPower * Math.cos(-robotPosition.rotation) - yPower * Math.sin(-robotPosition.rotation);
            double rotatedYPower = xPower * Math.sin(-robotPosition.rotation) + yPower * Math.cos(-robotPosition.rotation);

            // Calculate the power required to make the robot rotate in a specified direction.
            double rotationalPower = -rotationController.update(rotationalError);

            // Multiply all of the power values by the smoothingFactor. This allows us to gradually
            // accelerate up to max speed.
            rotatedXPower *= smoothingFactor * STRAFE_OFFSET;
            rotatedYPower *= smoothingFactor;
            rotationalPower *= smoothingFactor;

            // Calculate the value that all powers have to be divided by in order to ensure that they
            // maintain a consistent ratio and can move in the desired direction.
            double denominator = Math.max(Math.abs(rotatedXPower) + Math.abs(rotatedYPower) + Math.abs(rotationalPower), 1);

            // Calculate the power for each individual motor.
            double rightFrontPower = (rotatedYPower - rotatedXPower - rotationalPower) / denominator;
            double rightBackPower = (rotatedYPower + rotatedXPower - rotationalPower) / denominator;
            double leftFrontPower = (rotatedYPower + rotatedXPower + rotationalPower) / denominator;
            double leftBackPower = (rotatedYPower - rotatedXPower + rotationalPower) / denominator;

            // Use the pre-existing function to apply the power to the wheels.
            setDrivePower(rightFrontPower, rightBackPower, leftFrontPower, leftBackPower);

            // Display useful data to user:
            myOpMode.telemetry.addData("X Error: ", xDistance);
            myOpMode.telemetry.addData("Y Error: ", yDistance);
            myOpMode.telemetry.addData("X Target: ", targetPosition.x);
            myOpMode.telemetry.addData("X Target: ", targetPosition.y);

            // Update robot's position
            coordinateSystem.updateRobotPosition(
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

            // Show the driver the robot's current position
            displayRobotPosition();
        }

        setDrivePower(0, 0, 0, 0);

    }

    /**
     * Rotates the robot so that its facing the provided direction. Accounts for any disturbances that
     * might disrupt this process by using a PID controller, which allows us to respond to these disturbances.
     *
     * @param targetAnge The angle (in radians) you want the robot to be facing.
     */
    public void rotateTo(double targetAnge) {

        // Create a TurnToPIDController to handle the robot's rotation.
        TurnToPIDController rotationController = new TurnToPIDController(0.0135, 0, 0.003);

        // Calculate the difference between the robot's current
        double error = coordinateSystem.getDistanceToRotation(targetAnge);

        // Continues rotating until the robot is within 1 degree of it's targetAngle or until
        // the opMode is stopped.
        while (myOpMode.opModeIsActive() && Math.abs(error) > Math.toRadians(1)) {

            // Update the error (Difference between targetPosition and currentPosition)
            error = coordinateSystem.getDistanceToRotation(targetAnge);

            // Calculate the power required to turn the robot.
            double targetPower = rotationController.update(error);

            // Apply motor powers to each motor.
            setDrivePower(targetPower, targetPower, -targetPower, -targetPower);

            // Update robot's position
            coordinateSystem.updateRobotPosition(
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

            // Debug //
            myOpMode.telemetry.addLine("---PID Related Values---");
            myOpMode.telemetry.addData("Error (Radians) ", error);
            myOpMode.telemetry.addData("Error (Degrees) ", Math.toDegrees(error));

            // Show the driver the robot's current position
            displayRobotPosition();
        }

        // Stop the robot from moving
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Displays what position the robot is currently located at on the field on the Telemetry.
     */
    public void displayRobotPosition() {

        // Get the robot's position
        RobotPosition robotPosition = coordinateSystem.getPosition();

        // Convert the robot's rotation to degrees to make it easier for a human to understand.
        double robotRotationDegrees = Math.toDegrees(robotPosition.rotation);
        /*
        // Tell the user their current position.
        myOpMode.telemetry.addLine("---Settings---");
        myOpMode.telemetry.addData("DriveMode: ", current_drive_mode);
        myOpMode.telemetry.addData("Speed Multiplier", current_drive_mode.speedMultiplier());
        myOpMode.telemetry.addData("FieldCentricEnabled", fieldCentric);
        */


        myOpMode.telemetry.addLine("---PID Coefficients---");
        myOpMode.telemetry.addData("Kp ", pidController.getProportionalTerm());
        myOpMode.telemetry.addData("Ki ", pidController.getIntegralTerm());
        myOpMode.telemetry.addData("kd ", pidController.getDerivativeTerm());

        myOpMode.telemetry.addLine("---Motor Powers---");
        myOpMode.telemetry.addData("RF Power:", rightFrontDrive.getPower());
        myOpMode.telemetry.addData("RB Power:", rightBackDrive.getPower());
        myOpMode.telemetry.addData("LF Power:", leftFrontDrive.getPower());
        myOpMode.telemetry.addData("LB Power:", leftBackDrive.getPower());

        myOpMode.telemetry.addLine("---Robot Position---");
        myOpMode.telemetry.addData("Robot X", robotPosition.x);
        myOpMode.telemetry.addData("Robot Y", robotPosition.y);
        myOpMode.telemetry.addData("Robot Rotation (Radians)", robotPosition.rotation);
        myOpMode.telemetry.addData("Robot Rotation (Degrees)", robotRotationDegrees);
        myOpMode.telemetry.addData("Rotational Offset ", coordinateSystem.rotationalOffset);

        myOpMode.telemetry.update();
    }

    /**
     * Switch the robot from one mode to another, (allowing for the user to go slower or faster)
     *
     * @param drive_mode The drive mode the user wants to change the robot to.
     */
    public void setDriveMode(DriveMode drive_mode) {
        current_drive_mode = drive_mode;
    }

    /**
     * Toggles field centric drive on and off.
     */
    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    /**
     * Resets the IMU to it's original position. This helps counteract IMU drift.
     */
    public void resetIMU() {
        coordinateSystem.resetIMU();
    }
}