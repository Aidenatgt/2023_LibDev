package frc.robot.AidenLib;

/** A class which keeps track of time. */
public class Timer {
    private double lastTime, startTime;

    /**
     * Constructs a {@link Timer} object.
     */
    public Timer() {
        this.lastTime = System.currentTimeMillis() / 1000.0;
        this.startTime = System.currentTimeMillis() / 1000.0;
    };

    /**
     * @return The length of time since the last call of this method.
     */
    public double getDT() {
        double time = System.currentTimeMillis() / 1000.0;
        double dt = time - lastTime;
        lastTime = time;
        return dt;
    }

    /**
     * Starts/restarts a stopwatch.
     */
    public void start() {
        this.startTime = System.currentTimeMillis() / 1000.0;
    }

    /**
     * @return The current time into the stopwatch.
     */
    public double getDuration() {
        return System.currentTimeMillis() / 1000.0 - startTime;
    }
}
