package org.firstinspires.ftc.teamcode.RobotSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlide {

    private DcMotor linearSlide = null;
    private LinearSlideStage current_stage = LinearSlideStage.GROUND_STAGE;
    private LinearOpMode myOpMode = null;
    private int targetPosition = 0;

    // Get the opMode so that we can get hardware.
    public LinearSlide(LinearOpMode opMode) {myOpMode = opMode; }

    public void init(){

        // Initialize the hardware variables.
        linearSlide = myOpMode.hardwareMap.get(DcMotor.class, "linearSlide");

        // Setup linearSlide motor so that we can use encoders.
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(targetPosition);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the user that the linearSlide has been successfully initialized.
        myOpMode.telemetry.addData("->", "Linear Slide Successfully Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Adjusts the LinearSlide's stage which changes the target position of the motor.
     *
     * @param newStage;    The stage the user wants the LinearSlide to go up/down to.
     */
    public void setStage(LinearSlideStage newStage) {

        // Make sure we aren't trying to set the stage to the stage we are already on.
        if (current_stage == newStage) {
            return;
        }

        // Change the current stage.
        current_stage = newStage;

        // Move to new target position.
        targetPosition = current_stage.getPosition();
        linearSlide.setTargetPosition(targetPosition);
    }
}
