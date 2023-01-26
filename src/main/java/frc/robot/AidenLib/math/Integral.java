package frc.robot.AidenLib.math;

import frc.robot.AidenLib.Timer;

/** A class which approximates an integrated value with respect to time. */
public class Integral {
    
    protected double total;
    protected Timer timer = new Timer();

    /**
     * Constructs an {@link Integral} object.
     * @param C The initial value of the integral.
     */
    public Integral(double C) {
        this.total = C;
    }

    /**
     * Constructs an {@link Integral} object who's initial value is defaulted to <b>0.0</b>.
     */
    public Integral() {
        this(0.0);
    }

    /**
     * This is a more accurate way to approximate an integral than a Riemann Sum, but it requires an extra piece of input.
     * @param v1 The previous <b>v</b> value from the last robot loop.
     * @param v2 The current <b>v</b> value of the current robot loop.
     * @param dt The time between robot loops in seconds.
     * @return The approximate area of the function in the interval of width <b>dt</b>.
     */
    public static double linearSum(double v1, double v2, double dt) {
        return v1 * dt + (v2 - v1) * dt / 2.0;
    }

    /**
     * This is a less accurate way to approximate an integral than a linear approximation, but it only requires one <b>v</b>.
     * @param v The value to be integated.
     * @param dt The time between robot loops in seconds.
     * @return The approximate area of the function in the interval of width <b>dt</b>.
     */
    public static double riemannSum(double v, double dt) {
        return v * dt;
    }

    /**
     * This method employs a Riemann Sum to approximate the integral of v. For a linear approximation, use the {@link LinearIntegral} class.
     * <p>
     * Call this in a method that loops (such as a periodic method).
     * </p>
     * <p>
     * To get the value of the integral call the <b>get()</b> method.
     * </p>
     * @param v The value which is being integrated with respect to time.
     */
    public void update(double v) {
        total += riemannSum(v, timer.getDT());
    }

    /**
     * @return The current value of the integral.
     */
    public double get() {
        return total;
    }

    @Override
    public String toString() {
        return String.format("%f", get());
    }
}
