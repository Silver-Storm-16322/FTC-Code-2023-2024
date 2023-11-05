# Vector2 #
The `Vector2` class serves as a container for X and Y position values.

## Fields ##
### X ###
`private double x`

Represents the X coordinate in the `Vector2`.

### Y ###
`private double y`

Represents the Y coordinate in the `Vector2`.

## Methods ##
### addValues() ###
`public void addValues(Vector2 otherVector)`

This method combines the values of another `Vector2` with this vector.

### subtractValue() ###
`public void subtractValue(Vector2 otherVector)`

This method subtracts the values of a `Vector2` from this vector.

### divideValues() ###
`public void divideValues(Vector2 otherVector)`

Divides this vector's values by the corresponding values from the specified `Vector2`.

### divideBy() ###
`public void divideBy(double divisor)`

Divides all of the vector's values by the specified double.

### multiplyValues() ###
`public void multiplyBy(Vector2 otherVector)`

Multiplies this vector's values by the corresponding values from the specified `Vector2`.

### multiplyBy() ###
`public void multiplyBy(double multiplier)`

Multiplies all of the vector's values by the specified double.

### rotateVector() ###
`public void rotateVector(double angle)`

Rotates the vector around its origin by the specified angle in radians.