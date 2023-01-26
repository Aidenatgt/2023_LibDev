package frc.robot.AidenLib.math;

/**
 * Class to represent data input to the {@link KalmanFilter}.
 */
public class Data {
    public double value, stDev;

    /**
     * Constructs an instance of the {@link Data} class to be used in the {@link KalmanFilter} class.
     * @param value The value of the measurement taken.
     * @param stDev The expected standard deviation of the method of measurement.
     */
    public Data(double value, double stDev) {
        this.value = value;
        this.stDev = stDev;
    }
}