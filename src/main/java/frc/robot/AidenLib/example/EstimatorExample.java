package frc.robot.AidenLib.example;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.AidenLib.control.PoseEstimator3D;
import frc.robot.AidenLib.control.RotationEstimator3D;
import frc.robot.AidenLib.math.Data;

public class EstimatorExample {

    public static final double imuStdDev = 0.1;
    public static final double driveStDev = 0.1;

    private RotationEstimator3D rotEstimator = new RotationEstimator3D(imuStdDev);
    private PoseEstimator3D poseEstimator = new PoseEstimator3D(driveStDev);

    public void periodic(Drivetrain drive, Camera cam) {
        Pose3d camPose = cam.getPose();
        Rotation3d camRot = camPose.getRotation();
        double[] stDevs = cam.getStdDevs();
        Rotation3d rotEstimate = rotEstimator.estimate(drive.getRotation(),
            new Data[] {new Data(camRot.getZ(), stDevs[5])},
            new Data[] {new Data(camRot.getY(), stDevs[4])},
            new Data[] {new Data(camRot.getX(), stDevs[3])}
        );

        Pose3d poseEstimate = poseEstimator.estimate(drive.getChassisSpeeds(), rotEstimate,
        new Data[] {new Data(camPose.getX(), stDevs[2])},
        new Data[] {new Data(camPose.getY(), stDevs[1])},
        new Data[] {new Data(camPose.getZ(), stDevs[0])}
        );
    }
}
