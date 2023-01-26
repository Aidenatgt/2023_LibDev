package frc.robot.AidenLib.control;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.AidenLib.math.ParametricKalmanFilter;
import frc.robot.AidenLib.math.ParametricKalmanFilter.InvalidParamterSize;
import frc.robot.AidenLib.Timer;
import frc.robot.AidenLib.math.Data;
import frc.robot.AidenLib.math.Integral;

/** A class which works similarly to {@link Odometry3D} but in addition to drivetrain data it also fuses other measures of the robot position. */
public class PoseEstimator3D {
    private ParametricKalmanFilter filter;
    private Translation3d lastSpeeds;
    private Pose3d lastEstimate;
    private double driveStDev;
    private Timer timer;

    /**
     * Constructs a {@link PoseEstimator3D} object.
     * @param initialPose The initial position of the robot in <b>m</b>.
     * @param driveStDev The standard deviation of the drive data in <b>m</b>.
     */
    public PoseEstimator3D(Pose3d initialPose, double driveStDev) {
        this.filter = new ParametricKalmanFilter(6);
        this.lastSpeeds = new Translation3d();
        this.lastEstimate = initialPose;
        this.driveStDev = driveStDev;
        this.timer = new Timer();
    }

    /**
     * Constructs a {@link PoseEstimator3D} object with <b>new Pose3d()</b> as a default initial position.
     * @param driveStDev The standard deviation of the drive data error in <b>m per robot loop</b>.
     */
    public PoseEstimator3D(double driveStDev) {
        this(new Pose3d(), driveStDev);
    }

    /**
     * A method to calculate the optimal estimate of the robot's position based on drive odometry as well as other measurements.
     * <p>
     * The {@link Data} arrays may be different lengths.
     * </p>
     * @param speeds Robot relative speeds of the robot in <b>m/s</b>.
     * @param rotEstimate An estimate of the robot heading which may be directly supplied by an IMU but should be the result of the {@link RotationEstimator3D} class.
     * @param xMeasures An array of any length (including 0) representing measurements of the X position by {@link Data} objects in <b>m</b>.
     * @param yMeasures An array of any length (including 0) representing measurements of the Y position by {@link Data} objects in <b>m</b>.
     * @param zMeasures An array of any length (including 0) representing measurements of the Z position by {@link Data} objects in <b>m</b>.
     * @return The optimal estimate of the robot position based on the given measurements in <b>m</b>.
     */
    public Pose3d estimate(ChassisSpeeds speeds, Rotation3d rotEstimate, Data[] xMeasures, Data[] yMeasures, Data[] zMeasures) {
        double[] pVals = {0.0, 0.0, 0.0};

        Translation3d speedsT = new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0).rotateBy(rotEstimate);

        double dt = timer.getDT();
        double x = lastEstimate.getX() + Integral.linearSum(lastSpeeds.getX(), speedsT.getX(), dt);
        double y = lastEstimate.getY() + Integral.linearSum(lastSpeeds.getY(), speedsT.getY(), dt);
        double z = lastEstimate.getZ() + Integral.linearSum(lastSpeeds.getZ(), speedsT.getZ(), dt);

        lastSpeeds = new Translation3d(x, y, z);

        ArrayList<Data> xList = new ArrayList<Data>(Arrays.asList(xMeasures));
        ArrayList<Data> yList = new ArrayList<Data>(Arrays.asList(yMeasures));
        ArrayList<Data> zList = new ArrayList<Data>(Arrays.asList(zMeasures));

        xList.add(new Data(lastSpeeds.getX(), driveStDev));
        yList.add(new Data(lastSpeeds.getY(), driveStDev));
        zList.add(new Data(lastSpeeds.getZ(), driveStDev));

        try {
            pVals = filter.calculate(xList.toArray(Data[]::new), yList.toArray(Data[]::new), zList.toArray(Data[]::new));
        } catch (InvalidParamterSize e) {
            e.printStackTrace();
        }

        return new Pose3d(pVals[0], pVals[1], pVals[2], rotEstimate);
    }
}
