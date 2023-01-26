package frc.robot.AidenLib.example;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.AidenLib.Timer;
import frc.robot.AidenLib.math.Data;
import frc.robot.AidenLib.math.Integral;
import frc.robot.AidenLib.math.ParametricWeightedFusion;
import frc.robot.AidenLib.math.ParametricWeightedFusion.InvalidParamterSize;

public class IMUFusionExample {

    public static final double accelStDev = 0.1;
    public static final double driveStDev = 0.05;

    private double[] lastVelEstimate = {0.0, 0.0, 0.0};
    private Timer timer = new Timer();

    private ParametricWeightedFusion filter = new ParametricWeightedFusion(3);

    public void periodic(Drivetrain drive, IMU imu) {
        double[] accel = imu.getAccel();
        double dt = timer.getDT();

        double[] vel = {0.0, 0.0, 0.0};

        vel[0] = lastVelEstimate[0] + Integral.riemannSum(accel[0], dt);
        vel[1] = lastVelEstimate[1] + Integral.riemannSum(accel[1], dt);
        vel[2] = lastVelEstimate[2] + Integral.riemannSum(accel[2], dt);

        ChassisSpeeds speeds = drive.getChassisSpeeds();

        try {
            vel = filter.calculate(
                new Data[] {new Data(vel[0], accelStDev), new Data(speeds.vxMetersPerSecond, driveStDev)},
                new Data[] {new Data(vel[1], accelStDev), new Data(speeds.vyMetersPerSecond, driveStDev)},
                new Data[] {new Data(vel[2], accelStDev), new Data(0.0, driveStDev)}
            );
        } catch (InvalidParamterSize e) {
            e.printStackTrace();
        }

        lastVelEstimate = vel;
    }
}
