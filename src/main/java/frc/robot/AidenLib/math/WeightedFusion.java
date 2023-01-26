package frc.robot.AidenLib.math;

/**
 * Fuses data weighted on the standard deviation in the data from the "true" value.
 */
public class WeightedFusion {

    private double val;

    /**
     * Constructs an instance of the {@link WeightedFusion} class to start at a specified value.
     * @param startingVal This is the initial value of the value estimated by the filter. This may be valuable 
     * if an algorithm of relative measurement is made relative to the filter's estimation.
     */
    public WeightedFusion(double startingVal) {
        val = startingVal;
    }

    /**
     * Constructs an instance of the {@link WeightedFusion} class with a starting value of <b>0.0</b>.
     */
    public WeightedFusion() {
        this(0.0);
    }

    /**
     * This method applies the Kalman filter algorithm to a series of measurements.
     * @param estimates An array of {@link Data} objects which represent the series of measurements.
     * @return The estimate of the filter.
     */
    public double calculate(Data... estimates) {

        double numerator = 0.0;
        double denominator = 0.0;

        for (Data est : estimates) {
            numerator += Math.pow(est.stDev, -2.0) * est.value;
            denominator += Math.pow(est.stDev, -2.0);
        }

        val = numerator / denominator;

        return val;
    }

    /**
     * @return The most recent value estimated.
     */
    public double get() {
        return val;
    }
}
