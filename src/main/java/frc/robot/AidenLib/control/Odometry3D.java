package frc.robot.AidenLib.control;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.AidenLib.math.Derivative;
import frc.robot.AidenLib.math.Integral;
import frc.robot.AidenLib.math.LinearIntegral;

/** A class which keeps track of position in 3 dimensions which only relies on data from the drivetrain and an IMU. */
public class Odometry3D {
    
    private LinearIntegral xVal, yVal, zVal;
    private Integral yawVal, pitchVal, rollVal;
    private Derivative yawRate, pitchRate, rollRate;

    Pose3d pose;

    /**
     * Creates an {@link Odometry3D} object.
     * @param initialPose The initial position of the robot in <b>m</b>.
     */
    public Odometry3D(Pose3d initialPose) {
        pose = initialPose;
        xVal = new LinearIntegral(initialPose.getX());
        yVal = new LinearIntegral(initialPose.getY());
        zVal = new LinearIntegral(initialPose.getZ());
        Rotation3d rot = initialPose.getRotation();
        yawVal = new Integral(rot.getZ());
        pitchVal = new Integral(rot.getY());
        rollVal = new Integral(rot.getX());
        yawRate = new Derivative(rot.getZ());
        pitchRate = new Derivative(rot.getY());
        rollRate = new Derivative(rot.getX());
    }

    /**
     * Creates an {@link Odometry3D} object using <b>new Pose3d()</b> as the default initial position.
     */
    public Odometry3D() {
        this(new Pose3d());
    }

    /**
     * This method integrates the velocities of the robot in 3 dimensions to find the position of the robot.
     * @param speeds Robot relative speeds of the robot in <b>m/s</b>.
     * @param heading The rotation of the robot as supplied by an IMU.
     * @return The integrated position of the robot in <b>m</b>.
     */
    public Pose3d update(ChassisSpeeds speeds, Rotation3d heading) {
        yawVal.update(yawRate.getRate(heading.getZ()));
        pitchVal.update(pitchRate.getRate(heading.getY()));
        rollVal.update(rollRate.getRate(heading.getX()));

        Rotation3d rot = new Rotation3d(rollVal.get(), pitchVal.get(), yawVal.get());

        Translation3d speedsT = new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0).rotateBy(rot);

        xVal.update(speedsT.getX());
        yVal.update(speedsT.getY());
        zVal.update(speedsT.getZ());

        pose = new Pose3d(xVal.get(), yVal.get(), zVal.get(), rot);
        return pose;
    }

    /**
     * @return The most recent result of the odometry algorithm.
     */
    public Pose3d getPose() {
        return pose;
    }
}
