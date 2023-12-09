package org.firstinspires.ftc.teamcode.Utility.PositionDataTypes;

public class FieldPosition {

    public double x;
    public double y;
    public FieldPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Create a FieldPosition with the same values as the provided field position.
     *
     * @param position The position who's values you want to clone.
     */
    public FieldPosition(FieldPosition position) {
        this.x = position.x;
        this.y = position.y;
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

    /**
     * This method calculates the magnitude of the vector.
     *
     * @return Returns the magnitude of the vector.
     */
    public double getMagnitude() {

        // Calculate what each of the
        double xSquared = Math.pow(this.x, 2);
        double ySquared = Math.pow(this.y, 2);

        return Math.sqrt(xSquared + ySquared);
    }

    /**
     * Calculates the unit vector of this vector.
     *
     * @return Returns the unit vector of this FieldPosition.
     */
    public FieldPosition getUnitVector() {

        // Get this vector's magnitude
        double magnitude = this.getMagnitude();

        // Create a new FieldPosition with the same X and Y values.
        FieldPosition unitVector = new FieldPosition(this.x, this.y);
        unitVector.divideBy(magnitude);

        // Return the unit vector.
        return unitVector;
    }
}