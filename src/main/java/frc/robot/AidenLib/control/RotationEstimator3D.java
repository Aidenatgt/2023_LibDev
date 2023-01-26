package frc.robot.AidenLib.control;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.AidenLib.Timer;
import frc.robot.AidenLib.math.Derivative;
import frc.robot.AidenLib.math.Integral;
import frc.robot.AidenLib.math.ParametricKalmanFilter;
import frc.robot.AidenLib.math.Data;

/** A class which estimates the robot rotation in 3D space based on imu data fused with other measurements. */
public class RotationEstimator3D {
    private ParametricKalmanFilter filter;

    private Rotation3d lastEstimate;

    private Derivative xRate, yRate, zRate;
    private Timer timer;

    private double imuStdDev;

    /**
     * Constructs a {@link RotationEstimator3D} object.
     * @param imuStDev The standard deviation of the IMU error in <b>radians per robot loop</b>.
     * @param initialRot The intial rotation of the robot.
     */
    public RotationEstimator3D(double imuStDev, Rotation3d initialRot) {
        this.lastEstimate = initialRot;
        this.filter = new ParametricKalmanFilter(3);
        this.xRate = new Derivative(initialRot.getX());
        this.yRate = new Derivative(initialRot.getY());
        this.zRate = new Derivative(initialRot.getZ());
        this.timer = new Timer();
        this.imuStdDev = imuStDev;
    }

    /**
     * Constructs a {@link RotationEstimator3D} object using <b>new Pose3d()</b> as the default initial position.
     * @param imuStDev The standard deviation of the IMU error in <b>radians per robot loop</b>.
     */
    public RotationEstimator3D(double imuStDev) {
        this(imuStDev, new Rotation3d());
    }

    /**
     * A method which fuses IMU data with other provided measurements.
     * <p>
     * This method provides an intended input to the <b>estimate()</b> method of the {@link PoseEstimator3D} class.
     * </p>
     * @param imuMeasure The data provided from the IMU in the form of a {@link Rotation3d} object.
     * @param yawMeasures An array of any length (including 0) representing measurements of the yaw rotation by {@link Data} objects in <b>radians</b>.
     * @param pitchMeasures An array of any length (including 0) representing measurements of the pitch rotation by {@link Data} objects in <b>radians</b>.
     * @param rollMeasures An array of any length (including 0) representing measurements of the roll rotation by {@link Data} objects in <b>radians</b>.
     * @return The optimal estimate of the robot rotation.
     */
    public Rotation3d estimate(Rotation3d imuMeasure, Data[] yawMeasures, Data[] pitchMeasures, Data[] rollMeasures) {
        double[] pVals = {0.0, 0.0, 0.0};

        double dt = timer.getDT();

        imuMeasure = new Rotation3d(
            lastEstimate.getX() + Integral.riemannSum(xRate.getRate(imuMeasure.getX()), dt),
            lastEstimate.getY() + Integral.riemannSum(yRate.getRate(imuMeasure.getY()), dt),
            lastEstimate.getZ() + Integral.riemannSum(zRate.getRate(imuMeasure.getZ()), dt)
        );

        ArrayList<Data> xList = new ArrayList<Data>(Arrays.asList(rollMeasures));
        ArrayList<Data> yList = new ArrayList<Data>(Arrays.asList(pitchMeasures));
        ArrayList<Data> zList = new ArrayList<Data>(Arrays.asList(yawMeasures));

        xList.add(new Data(imuMeasure.getX(), imuStdDev));
        yList.add(new Data(imuMeasure.getY(), imuStdDev));
        zList.add(new Data(imuMeasure.getZ(), imuStdDev));

        try {
            pVals = filter.calculate(xList.toArray(Data[]::new), yList.toArray(Data[]::new), zList.toArray(Data[]::new));
        } catch (Exception e) {
            e.printStackTrace();
        }

        return new Rotation3d(pVals[0], pVals[1], pVals[2]);
    }
}