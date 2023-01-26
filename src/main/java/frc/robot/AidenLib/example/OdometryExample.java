package frc.robot.AidenLib.example;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.AidenLib.control.Odometry3D;

public class OdometryExample {

    private Odometry3D odometry = new Odometry3D();

    public void periodic(Drivetrain drive) {
        Pose3d pose = odometry.update(drive.getChassisSpeeds(), drive.getRotation());
    }
}
