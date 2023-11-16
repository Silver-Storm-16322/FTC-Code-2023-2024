package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnToPIDController {
    private double kp, ki, kd;
    private double accumulatedError, previousError = 0;
    private ElapsedTime timer = new ElapsedTime();

    /**
     * Sets the values for all of the coefficients that will be used to calculate the motor power
     * based off of the difference between the targetPosition and currentPosition.
     *
     * @param proportionalCoefficient Helps prevents overshooting the target
     * @param integralCoefficient Helps prevent steady state error. (Helps overcome obstacles are
     *                            consistently blocking it off.
     * @param derivativeCoefficient Help prevent oscillation (Value changing rapidly back and forth).
     */
    public TurnToPIDController(double proportionalCoefficient, double integralCoefficient, double derivativeCoefficient) {
        this.kp = proportionalCoefficient;
        this.ki = integralCoefficient;
        this.kd = derivativeCoefficient;
    }

    /**
     * Calculates the motor power based on the difference between the currentPosition and the
     * targetPosition via the use of various tuned values to help account for any disturbances the
     * robot may encounter while turning.
     *
     * @param error The difference between the currentPosition and the targetPosition.
     * @return Returns
     */
    public double update(double error) {

        // Proportional Calculations //
        error = Math.toDegrees(error);

        // Integral Calculations //
        accumulatedError += error * timer.seconds();

        // Reset our accumulatedError when we get to our desired margin of error. (Helps stop the
        // robot because we will almost never land exactly on our desired direction.
        if (error < Math.toRadians(1)) {
            accumulatedError = 0;
        }

        // Compensated for if the robot gets pushed so much that its more efficient to turn in the
        // opposite direction.
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        // Derivative Calculations //
        double derivative = (error - previousError) / timer.seconds();
        previousError = error;

        // Reset timer so the next time this method is run the calculations are accurate.
        timer.reset();

        // Motor power calculations //
        // Note: tanh() limits values between -1 and 1.
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(
                kp * error + (ki * accumulatedError) + (kd * derivative));

        // Output the calculated result
        return motorPower;
    }
}