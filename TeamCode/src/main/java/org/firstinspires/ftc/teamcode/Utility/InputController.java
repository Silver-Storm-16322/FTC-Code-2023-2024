package org.firstinspires.ftc.teamcode.Utility;

public class InputController {
    private double smoothingFactor;
    private int scalingFactor = 0;
    private double previousInput = 0;

    public InputController(double smoothingFactor, int scalingFactor) {
        this.smoothingFactor = smoothingFactor;
        this.scalingFactor = scalingFactor;
    }

    /**
     * This method helps prevent a value from increasing too fast
     *
     * @param input The value you want to prevent from increasing too quickly.
     * @return Returns a smoothed out version of your input
     */
    public double smoothInput(double input) {
        double smoothedInput = previousInput + smoothingFactor * (input - previousInput);
        previousInput = smoothedInput;
        return smoothedInput;
    }

    /**
     * This method
     * @param input The value you want to scale.
     * @return Returns the input to the power of the provided scaling factor.
     */
    private double scaleInput(double input) {
        return Math.pow(input, scalingFactor);
    }

    /**
     * This method helps give the driver more control at slow speeds and adds some acceleration and
     * deceleration to reduce slippage.
     *
     * @param input The value you want to scale and smooth out.
     * @return Returns the smoothed putout the power of the scaling factor.
     */
    public double smoothScaleInput(double input) {
        double scaledInput = scaleInput(input);
        return smoothInput(scaledInput);
    }
}
