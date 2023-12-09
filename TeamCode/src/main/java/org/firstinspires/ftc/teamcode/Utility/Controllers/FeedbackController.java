package org.firstinspires.ftc.teamcode.Utility.Controllers;

public class FeedbackController {

    double k = 0;

    /**
     * Set the k coefficient for this FeedbackController
     *
     * @param k A value that you will have to tune. (Higher values will make you get to your target
     *          faster.
     */
    public FeedbackController(double k) {
        this.k = k;
    }


    /**
     * Given a target value and the current value, calculate the feedback response.
     *
     * @param reference The value we want to move towards
     * @param state The value we are currently at.
     * @return Returns a value that will be applied as a motor power.\ ot help us reach the reference.
     */
    public double calculateFeedback(double reference, double state) {

        // Calculate the difference between the reference and the state.
        double error = reference - state;

        // Calculate the return value. This can be used as a motor power to help us get to the reference.
        double u = k * error;

        // Return the calculated feedback.
        return u;
    }
}
