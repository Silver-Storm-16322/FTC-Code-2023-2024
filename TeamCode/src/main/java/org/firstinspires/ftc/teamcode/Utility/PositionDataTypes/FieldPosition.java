package org.firstinspires.ftc.teamcode.Utility.PositionDataTypes;

public class FieldPosition {

    public double x;
    public double y;
    public FieldPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * This method combines the values of another Vector2 with this vector.
     * Adds the X and Y values from the specified Vector2 to this vector.
     *
     * @param otherVector The Vector2 instance whose values will be added to this vector.
     */
    public void addValues(FieldPosition otherVector) {
        this.x += otherVector.x;
        this.y += otherVector.y;
    }

    /**
     * This method subtracts the values of a Vector2 from this vector.
     * Subtracts the X and Y values from the specified Vector2 from this vector.
     *
     * @param otherVector The Vector2 instance who's values will be subtracted from this vector.
     */
    public void subtractValues(FieldPosition otherVector) {
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
    public void divideValues(FieldPosition otherVector) {
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
    public void multiplyValues(FieldPosition otherVector) {
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