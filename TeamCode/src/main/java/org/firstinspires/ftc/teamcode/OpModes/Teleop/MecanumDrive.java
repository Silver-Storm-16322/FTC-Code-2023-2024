package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.DriveMode;
import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.InputController;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Controllers.TurnToPIDController;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Mecanum Drive", group="Mecanum Drive Trains 2023-2024")
public class MecanumDrive extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    InputController leftStickYController = new InputController(.2, 3);
    InputController leftStickXController = new InputController(.2, 3);
    InputController rightStickXController = new InputController(.2, 3);
    TurnToPIDController rotationController = new TurnToPIDController(0.015, 0, 0.003);
    Timer timer = new Timer(2000, TimeUnit.MILLISECONDS);
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    boolean rotationalCorrection = false;
    double lockedAngle = 0;

    @Override
    public void runOpMode() {

        // Initialize the robot
        robot.init();

        // Start the timer to allow for it's use later on.
        timer.start();

        // Wait for the driver to press the play button.
        waitForStart();

        // Runs while the opmode is active.
        while (opModeIsActive()) {

            // Copy the values from the current gamepad to previous gamepad.
            // This allows us to run each input only once.
            previousGamepad.copy(currentGamepad);

            // Set current gamepad to whatever the gamepad's input currently is.
            currentGamepad.copy(gamepad1);

            // Open and close the manipulator on command.
            if (currentGamepad.x) {
                robot.manipulator.close();
            }
            if (currentGamepad.a) {
                robot.manipulator.open();
            }

            // Toggles rotational correction on / off
            if (currentGamepad.guide && !previousGamepad.guide) {

                // Toggle rotational correction on/off depending on if it is currently enabled or not.
                if (rotationalCorrection) {
                    rotationalCorrection = false;
                } else {

                    // Set the value to true
                    rotationalCorrection = true;

                    // Reset the PID Controller to ensure accuracy.
                    rotationController.reset();

                    // Set lockedAngle to the current rotation of the robot.
                    lockedAngle = robot.driveTrain.coordinateSystem.getPosition().rotation;
                }
            }

            // Toggles field centric drive on/off if the button isn't being held down.
            if (currentGamepad.start && !previousGamepad.start) {

                // Turn field centric drive on or off depending om what mode is currently in.
                robot.driveTrain.toggleFieldCentric();
            }

            // Resets the IMU to help counteract IMU drift.
            if (currentGamepad.back) {

                // Reset the IMU
                robot.driveTrain.resetIMU();

                // If rotational correction mode is enabled, then reset the locked angle so that the
                // robot doesn't immediately rotate to an undesired position when the IMU is reset.
                if (rotationalCorrection) {
                    lockedAngle = robot.driveTrain.coordinateSystem.getPosition().rotation;
                }
            }

            // Sets the drive mode to precise drive. (Goes 4* slower than default)
            if (currentGamepad.left_bumper) {
                robot.driveTrain.setDriveMode(DriveMode.PRECISE_DRIVE);
            }


            // Sets the drive mode to its default mode.
            if (currentGamepad.y) {
                robot.driveTrain.setDriveMode(DriveMode.DEFAULT_DRIVE);
            }

            // These buttons are more difficult to press meaning they are less likely to be accidentally hit
            // during stressful moments.
            if (currentGamepad.right_trigger > 0.5 || currentGamepad.left_trigger > 0.5) {
                robot.paperAirplaneLauncher.launchPaperAirplane();
            }

            /*
            Smooth out the inputs to allow for smooth deceleration and acceleration. Additionally,
            this allows for more precise movements when the joystick is moved less, leading to more
            veritable controls.
             */
            double smoothedLeftStickY = leftStickYController.smoothScaleInput(currentGamepad.left_stick_y);
            double smoothedLeftStickX = leftStickXController.smoothScaleInput(currentGamepad.left_stick_x);
            double smoothedRightStickX = rightStickXController.smoothScaleInput(currentGamepad.right_stick_x);

            // If the driver has rotationalCorrection enabled, then help ensure the robot always faces
            // the desired direction.
            if (rotationalCorrection) {

                /*
                Get the absolute value of the amount we want to rotate (how far the right joystick
                is moved left/right. This will be used in the below calculations to ensure the robot
                faces the direction the driver want's it to be facing.
                 */
                double absoluteRotationalPower = Math.abs(smoothedRightStickX);

                /*
                If we are trying to turn the robot with less than 0.01 power (the driver doesn't
                want the robot to be rotated) then make sure we are facing the desired direction.
                Otherwise, set lockedAngle to the current rotation of the robot, since that is the
                direction the driver wants the robot to be facing.
                 */
                if (!(absoluteRotationalPower > .01)) {

                    // Get the distance the robot is from the direction we want it to be facing.
                    double rotationalError = robot.driveTrain.coordinateSystem.getDistanceToRotation(lockedAngle);

                    // If our robot is rotated more than 1 degree away from the direction we want to be
                    // facing, then apply the necessary motor powers to face in the desired direction.
                    if (Math.abs(rotationalError) > Math.toRadians(1)) {
                        smoothedRightStickX = -rotationController.update(rotationalError);
                    }
                } else {

                    // Update the locked angle so that the robot will be facing in the direction the
                    // driver wants.
                    lockedAngle = robot.driveTrain.coordinateSystem.getPosition().rotation;
                }
            }

            // Drive the robot using the smoothed out values.
            robot.driveTrain.driveRobot(-smoothedLeftStickY, smoothedLeftStickX, smoothedRightStickX);
        }
    }
}