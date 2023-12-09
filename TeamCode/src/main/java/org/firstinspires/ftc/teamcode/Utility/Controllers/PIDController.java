package org.firstinspires.ftc.teamcode.Utility.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kp, ki, kd;
    private double accumulatedError, previousError = 0;
    private ElapsedTime timer = new ElapsedTime();

    /**
     * Sets the values for all of the coefficients that will be used to calculate the motor power
     * based off of the difference between the reference and currentPosition.
     *
     * @param proportionalCoefficient Helps prevents overshooting the target
     * @param integralCoefficient Helps prevent steady state error. (Helps overcome obstacles are
     *                            consistently blocking it off.
     * @param derivativeCoefficient Help prevent oscillation (Value changing rapidly back and forth).
     */
    public PIDController(double proportionalCoefficient, double integralCoefficient, double derivativeCoefficient) {
        this.kp = proportionalCoefficient;
        this.ki = integralCoefficient;
        this.kd = derivativeCoefficient;
    }

    /**
     * Set's all of this PIDController's coefficients by cloning the values from another PIDController.
     *
     * @param pidController The PIDController who's values you want to initialize this PIDController's
     *                      values with.
     */
    public PIDController(PIDController pidController) {
        this.kp = pidController.getProportionalTerm();
        this.ki = pidController.getIntegralTerm();
        this.kd = pidController.getDerivativeTerm();
    }

    /**
     * Resets all of the values used by the PID controller so that the same controller can be used
     * multiple times.
     */
    public void reset() {
        this.accumulatedError = 0;
        this.previousError = 0;
        timer.reset();
    }

    /**
     * Calculates a motor power based on a the difference between the current input and the reference
     * by using the numerous tuned values that help account for any disturbances the robot may encounter
     * while moving a mechanism to a desired position.
     *
     * @param reference The value you want to be at.
     * @param currentInput The value you are currently at.
     * @return Returns a value between -1 and 1 that can be used as a DCMotor power.
     */
    public double update(double reference, double currentInput) {

        // Proportional Calculations //
        double error = reference - currentInput;

        // Integral Calculations //
        accumulatedError += error * timer.seconds();

        // Compensated for if it becomes more efficient for a mechanism to move in the opposite direction.
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        // Derivative Calculations //
        double derivative = (error - previousError) / timer.seconds();
        previousError = error;

        // Reset our accumulatedError when we get to our desired margin of error. (Helps stop the
        // robot because we will almost never land exactly on our desired direction.
        if (Math.abs(error) < .1) {
            accumulatedError = 0;
        }

        // Reset timer so the next time this method is run the calculations are accurate.
        timer.reset();

        // Calculate and return the output.
        return .05 * Math.signum(error) + .95 * Math.tanh( kp * error + (ki * accumulatedError) + (kd * derivative));
    }

    /**
     * Sets this PIDController's proportional term.
     *
     * @param newKp The value you want to set this PID Controller's proportional term to.
     */
    public void setKp(double newKp) {
        this.kp = newKp;
    }

    /**
     * Sets this PIDController's integral term.
     *
     * @param newKi The value you want to set this PID Controller's integral term to.
     */
    public void setKi(double newKi) {
        this.ki = newKi;
    }

    /**
     * Sets this PIDController's derivative term.
     *
     * @param newKd The value you want to set this PID Controller's derivative term to.
     */
    public void setKd(double newKd) {
        this.kd = newKd;
    }

    /**
     * Gets this PIDController's proportional term.
     * @return Returns this PIDController's proportional term.
     */
    public double getProportionalTerm() {
        return this.kp;
    }

    /**
     * Gets this PIDController's integral term.
     * @return Returns this PIDController's integral term.
     */
    public double getIntegralTerm() {
        return this.ki;
    }

    /**
     * Gets this PIDController's derivative term.
     * @return Returns this PIDController's derivative term.
     */
    public double getDerivativeTerm() {
        return this.kd;
    }
}
