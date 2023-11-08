package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.LinearSlideStage;

public class LinearSlide {

    private DcMotor linearSlide = null;
    private LinearSlideStage current_stage = LinearSlideStage.GROUND_STAGE;
    private int targetPosition = 0;

    // Get the opMode so that we can get hardware.
    public void init(HardwareMap hardwareMap, Telemetry telemetry){

        // Initialize the hardware variables.
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");

        // Setup linearSlide motor so that we can use encoders.
        // Note: We have to set a target position before we can tell the motors to run to a position
        // because if we don't it will error out.
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(targetPosition);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the user that the linearSlide has been successfully initialized.
        telemetry.addData("->", "Linear Slide Successfully Initialized");
    }

    /**
     * Adjusts the LinearSlide's stage which changes the target position of the motor.
     *
     * @param newStage The stage the user wants the LinearSlide to go up/down to.
     */
    public void setStage(LinearSlideStage newStage) {

        // Make sure we aren't trying to set the stage to the stage we are already at / moving towards.
        if (current_stage == newStage) {
            return;
        }

        // Change the current stage.
        current_stage = newStage;

        // Move the linear slides to the new slide position.
        targetPosition = current_stage.getPosition();
        linearSlide.setTargetPosition(targetPosition);
        linearSlide.setPower(1);
    }
}
