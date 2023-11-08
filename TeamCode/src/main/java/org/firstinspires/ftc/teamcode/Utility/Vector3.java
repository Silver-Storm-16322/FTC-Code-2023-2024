package org.firstinspires.ftc.teamcode.Utility;

public class Vector3 extends Vector2 {
    public double rotation;

    public Vector3(double x, double y, double rotation) {
        super(x, y);
        this.rotation = rotation;
    }

    /**
     * This method combines the values of another Vector3 with this Vector3 instance.
     * Adds the X, Y, and Rotation values from the specified Vector 3 to this Vector3.
     *
     * @param otherVector The Vector3 instance whose values will be added to this vector.
     */
    public void addValues(Vector3 otherVector) {
        this.x += otherVector.x;
        this.y += otherVector.y;
        this.rotation += otherVector.rotation;
    }

    /**
     * This method subtracts the values of a Vector3 from this vector.
     * Subtracts the X, Y, and Rotation values from the specified Vector3 from this vector.
     *
     * @param otherVector The Vector3 instance who's values will be subtracted from this vector.
     */
    public void subtractValues(Vector3 otherVector) {
        this.x -= otherVector.x;
        this.y -= otherVector.y;
        this.rotation -= otherVector.rotation;
    }

    /**
     * This method divides this vector's values by those from the specified Vector3.
     * Divides this vector's X, Y, and Rotation values by those from the specified Vector3.
     *
     * @param otherVector The Vector3 instance containing the values that this vector's values will
     *                    be divided by.
     */
    public void divideValues(Vector3 otherVector) {
        this.x /= otherVector.x;
        this.y /= otherVector.y;
        this.rotation /= otherVector.rotation;
    }

    /**
     * Divides all of the vector's values by the specified double.
     *
     * @param divisor The number you want to divide all of the vector's values by.
     */
    @Override
    public void divideBy(double divisor) {
        this.x /= divisor;
        this.y /= divisor;
        this.rotation /= divisor;
    }

    /**
     * This method multiplies this vector's values by those from the specified Vector3.
     * Multiplies this vector's X, Y, and Rotation values by those from the specified Vector3.
     *
     * @param otherVector The Vector3 instance containing the values that this vector's values will
     *                    be multiplied by.
     */
    public void multiplyValues(Vector3 otherVector) {
        this.x *= otherVector.x;
        this.y *= otherVector.y;
        this.rotation *= otherVector.rotation;
    }

    /**
     * Multiplies all of the vector's values by the specified double.
     *
     * @param multiplier The number you want to multiply all of the vectors values by.
     */
    @Override
    public void multiplyBy(double multiplier) {
        this.x *= multiplier;
        this.y *= multiplier;
        this.rotation *= multiplier;
    }

    /**
     * Rotates the vector around its origin by the specified angle in radians.
     * Additionally, updates the vector's rotation to account for the applied rotation angle.
     *
     * @param angle The angle (in radians) by which the vector should be rotated.
     */
    @Override
    public void rotateVector(double angle) {

        // Rotate this vector's X and Y values around its origin by the specified angle (In radians).
        super.rotateVector(angle);

        // Update the Vector3's rotation to account for the applied rotation angle.
        this.rotation -= angle;
    }
}