package org.firstinspires.ftc.teamcode.RobotSystems.Utility;

public class Vector2 {

    public double x;
    public double y;
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * This method combines the values of another Vector2 with this Vector2 instance.
     * Adds the X and Y values from the specified Vector2 to this Vector2.
     *
     * @param otherVector The Vector2 instance whose values will be added to this vector.
     */
    public void addValues(Vector2 otherVector) {
        this.x += otherVector.x;
        this.y += otherVector.y;
    }

    /**
     * This method subtracts the values of a Vector2 from this vector.
     * Subtracts the X and Y values from the specified Vector2 from this vector.
     *
     * @param otherVector The Vector2 instance who's values will be subtracted from this vector.
     */
    public void subtractValues(Vector2 otherVector) {
        this.x -= otherVector.x;
        this.y -= otherVector.y;
    }

    /**
     * This method divides this vector's values by those from the specified Vector2.
     * Divides this vector's X and Y values by those from the specified Vector2.
     *
     * @param otherVector The Vector2 instance containing the values that this vector's values will
     *                    be divided by.
     */
    public void divideValues(Vector2 otherVector) {
        this.x /= otherVector.x;
        this.y /= otherVector.y;
    }

    /**
     * Divides all of the vector's values by the specified double.
     *
     * @param divisor The number you want to divide all of the vector's values by.
     */
    public void divideBy(double divisor) {
        this.x /= divisor;
        this.y /= divisor;
    }

    /**
     * This method multiplies this vector's values by those from the specified Vector2.
     * Multiplies this vector's X and Y values by those from the specified Vector2.
     *
     * @param otherVector The Vector2 instance containing the values that this vector's values will
     *                    be multiplied by.
     */
    public void multiplyValues(Vector2 otherVector) {
        this.x *= otherVector.x;
        this.y *= otherVector.y;
    }

    /**
     * Multiplies all of the vector's values by the specified double.
     *
     * @param multiplier The number you want to multiply all of the vectors values by.
     */
    public void multiplyBy(double multiplier) {
        this.x *= multiplier;
        this.y *= multiplier;
    }

    /**
     * Rotates the vector around its origin by the specified angle in radians.
     *
     * @param angle The angle (in radians) by which the vector should be rotated.
     */
    public void rotateVector(double angle) {

        // Calculate the new coordinates after rotation.
        double rotatedX = this.x * Math.cos(-angle) - this.y * Math.sin(-angle);
        double rotatedY = this.x * Math.sin(-angle) + this.y * Math.cos(-angle);

        // Update the vector's coordinates after rotation.
        this.x = rotatedX;
        this.y = rotatedY;
    }
}