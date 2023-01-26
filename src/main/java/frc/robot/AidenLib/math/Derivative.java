package frc.robot.AidenLib.math;

import frc.robot.AidenLib.Timer;

/** A class which solves for the rate of change in a value. */
public class Derivative {
    private double lastVal;
    private Timer timer;

    /**
     * Constructs a new {@link Derivative} object.
     * @param x The initial value.
     */
    public Derivative(double x) {
        this.lastVal = x;
        this.timer = new Timer();
    }

    /**
     * Constructs a new {@link Derivative} object where the initial value is defaulted to <b>0.0</b>.
     */
    public Derivative() {
        this(0.0);
    }

    /**
     * Call this in a method that loops (such as a periodic method).
     * @param x The value who's rate of change is being solved for.
     * @return The rate of change of the input parameter.
     */
    public double getRate(double x) {
        double dx = x - lastVal;
        lastVal = x;
        return dx / timer.getDT();
    }
}
