package frc.robot.AidenLib.example;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface Camera {
    public Pose3d getPose();
    public Rotation3d getRotation();
    public double[] getStdDevs();
}
