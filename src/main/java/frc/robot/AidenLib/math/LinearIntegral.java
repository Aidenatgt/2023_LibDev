package frc.robot.AidenLib.math;

/** A class which approximates an integrated value with respect to time. */
public class LinearIntegral extends Integral {
    
    private double lastV;

    /**
     * Constructs an {@link LinearIntegral} object.
     * @param C The initial value of the integral.
     */
    public LinearIntegral(double C) {
        super(C);
        this.lastV = 0;
    }

    /**
     * Constructs an {@link LinearIntegral} object who's initial value is defaulted to <b>0.0</b>.
     */
    public LinearIntegral() {
        this(0.0);
    }

    /**
     * This method employs a linear approximation to approximate the integral of v. For a Riemann Sum approximation, use the {@link Integral} class.
     * <p>
     * Call this in a method that loops (such as a periodic method).
     * </p>
     * <p>
     * To get the value of the integral call the <b>get()</b> method.
     * </p>
     * @param v The value which is being integrated with respect to time.
     */
    @Override
    public void update(double v) {
        super.total += linearSum(lastV, v, super.timer.getDT());
        lastV = v;
    }
}
