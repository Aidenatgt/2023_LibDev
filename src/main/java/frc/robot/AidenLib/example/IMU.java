package frc.robot.AidenLib.example;

import edu.wpi.first.math.geometry.Rotation3d;

public interface IMU {
    public Rotation3d getHeading();
    public double[] getAccel();
}
