package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric Mecanum", group="Mecanum Drive Trains 2023-2024")
public class FieldCentricMecanumOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private IMU imu;

    @Override
    public void runOpMode() {

        // Initialize hardware variables;
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");

        // Reverse one side of te motors so that they all rotate in the same direction.
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set up the IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Note: Adjust values to fit robot.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        // Tell the user that the robot has been initialized
        telemetry.addData("Status","Initialized");
        telemetry.update();

        // Wait until someone presses play on the robot
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Use some math that I 100% didn't steal to
            // calculate how the mecanum wheels will work.
            double left_stick_y = -gamepad1.left_stick_y; // (The Y is reversed, so we have to multiply it by -1)
            double left_stick_x = gamepad1.left_stick_x;
            double right_stick_x = gamepad1.right_stick_x;

            // Reset the angle on the imu if the ___ button s pushed.
            // This is done to reduce/prevent unintended drifting.
            if (gamepad1.start) {
                imu.resetYaw();
            }

            double botAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction to counter the robots rotation.
            double rotX = left_stick_x * Math.cos(-botAngle) - left_stick_y * Math.sin(-botAngle);
            double rotY = left_stick_x * Math.cos(-botAngle) + left_stick_y * Math.sin(-botAngle);

            rotX = rotX * 1.1; // Counteract imperfect straifing.

            // Ensure that all powers maintain a consistent ratio.
            // This is required since all values are capped at 1.
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(right_stick_x), 1);
            double rightFrontPower = (rotY - rotX - right_stick_x) / denominator;
            double rightBackPower = (rotY + rotX - right_stick_x)/ denominator;
            double leftFrontPower = (rotY + rotX + right_stick_x) / denominator;
            double leftBackPower = (rotY - rotX + right_stick_x) / denominator;

            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
        }
    }
}
