package org.firstinspires.ftc.teamcode.Utility.Controllers;

public class FeedforwardsController {

    double kv = 0; // Velocity
    double ka = 0; // Acceleration

    /**
     *
     * @param kv The tuned value that helps control the velocity
     * @param ka The tuned value that helps control the acceleration
     */
    public FeedforwardsController(double kv, double ka) {
        this.kv = kv;
        this.ka = ka;
    }

    /**
     * This method returns a value that will help us reach the target velocity at a target
     * acceleration.
     *
     * @param targetVelocity The velocity we want to be moving at
     * @param targetAcceleration The amount we want to accelerate by to reach the target velocity.
     * @return Returns a value that will help us reach the target velocity at a target
     *         acceleration.
     */
    public double calculateFeedforwards(double targetVelocity, double targetAcceleration) {
        return ((kv * targetVelocity) + (ka * targetAcceleration));
    }
}
