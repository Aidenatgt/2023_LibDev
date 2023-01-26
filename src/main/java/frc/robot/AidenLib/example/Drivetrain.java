package frc.robot.AidenLib.example;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Drivetrain {
    public ChassisSpeeds getChassisSpeeds();
    public Pose3d getPose();
    public Rotation3d getRotation();
}
