package org.firstinspires.ftc.teamcode.OpModes.CalibrationOpModes.PIDCalibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utility.Controllers.SingleMotorPIDController;
import org.firstinspires.ftc.teamcode.Utility.FileEx;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

@TeleOp (name = "Drive To Position Tuner", group = "Calibration Op Modes")
public class DriveToPositionTuner extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    FileEx pidCoefficients = new FileEx("PIDCoefficients.txt");
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {

        // Initialize all of the robot's hardware.
        robot.init();

        // Get the drivetrain's PID Controller
        PIDController pidController = robot.driveTrain.pidController;

        // Set all of the PID Coefficients to the saved values.
        pidController.setKp(pidCoefficients.getValue("kp", Double.class));
        pidController.setKi(pidCoefficients.getValue("ki", Double.class));
        pidController.setKd(pidCoefficients.getValue("kd", Double.class));

        // Wait for the op mode to be started
        waitForStart();

        // Runs while the OpMode is active
        while (opModeIsActive()) {

            // Copy the values from the current gamepad to previous gamepad.
            // This allows us to run each input only once.
            previousGamepad.copy(currentGamepad);

            // Set current gamepad to whatever the gamepad's input currently is.
            currentGamepad.copy(gamepad1);

            // Test out autonomous rotation offset
            if (currentGamepad.guide && !previousGamepad.guide) {
                robot.driveTrain.coordinateSystem.setRobotPosition(new RobotPosition(0, 0, 0), Math.PI);
            }

            // Drive To Positions
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(0, 24, 0));
            }
            if (gamepad1.dpad_down && !previousGamepad.dpad_down) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(0, 0, Math.toRadians(-90)));
            }
            if (gamepad1.dpad_left && !previousGamepad.dpad_left) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(-24, 0, 0));
            }
            if (gamepad1.dpad_right && !previousGamepad.dpad_right) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(24, 0, 0));
            }

            // Change PID Coefficients //
            // Proportional Term
            if (currentGamepad.x && !previousGamepad.x) {

                // Update the PID Coefficient Value.
                pidController.setKp(pidController.getProportionalTerm() + 0.01);

                // Save the updated value to the file.
                pidCoefficients.addData("kp", pidController.getProportionalTerm());
            }
            if (currentGamepad.a && !previousGamepad.a) {

                // Update the PID Coefficient Value.
                pidController.setKp(pidController.getProportionalTerm() - 0.01);

                // Save the updated value to the file.
                pidCoefficients.addData("kp", pidController.getProportionalTerm());
            }

            // Integral Term
            if (currentGamepad.start && !previousGamepad.start) {

                // Update the PID Coefficient Value.
                pidController.setKi(pidController.getIntegralTerm() + 0.01);

                // Save the updated value to the file.
                pidCoefficients.addData("ki", pidController.getIntegralTerm());
            }
            if (currentGamepad.back && !previousGamepad.back) {

                // Update the PID Coefficient Value.
                pidController.setKi(pidController.getIntegralTerm() - 0.01);

                // Save the updated value to the file.
                pidCoefficients.addData("ki", pidController.getIntegralTerm());
            }
            // Derivative Term
            if (currentGamepad.y && !previousGamepad.y) {

                // Update the PID Coefficient Value.
                pidController.setKd(pidController.getDerivativeTerm() + 0.01);

                // Save the updated value to the file.
                pidCoefficients.addData("kd", pidController.getDerivativeTerm());
            }
            if (currentGamepad.b && !previousGamepad.b) {

                // Update the PID Coefficient Value.
                pidController.setKd(pidController.getDerivativeTerm() - 0.01);

                // Save the updated value to the file.
                pidCoefficients.addData("kd", pidController.getDerivativeTerm());
            }

            // Display useful information to the driver:
            robot.driveTrain.displayRobotPosition();
        }
    }
}
