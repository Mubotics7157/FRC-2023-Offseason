package frc.robot.util.Shooting;

import java.util.Arrays;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShotGenerator {

    private static ShotGenerator instance = new ShotGenerator();

    public static ShotGenerator getInstance(){
        return instance;
    }
    public class ShooterSetpoint{

        public final double shooterRPM;
        public final double hoodAngle;

        public ShooterSetpoint(double shooterRPM, double hoodAngle){
            this.shooterRPM = shooterRPM;
            this.hoodAngle = hoodAngle;
        }
    }

    //log data as {shooter wheel rpm, distance it made it in}
    private Double[][] wheelRPM = {
        {1.81,2.2,2.41,2.92, 3.03,3.27}, //3.96 last
        {1315d,1445d,1525d,1675d, 1755d,1820d}//#region} //3250d last
    };

    //log data as {hood angle (degrees), distance we made it in}
    private Double[][] hoodAngle = {
        {1.81,2.2,2.41,2.92, 3.03, 3.27}, //3.96 last
        {1d,.94,.92,.825, 0.8,.725} //.025 last
    };

    SplineInterpolator shooterInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(wheelRPM[0]), Arrays.asList(wheelRPM[1]));
    SplineInterpolator hoodInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(hoodAngle[0]), Arrays.asList(hoodAngle[1]));

    InterpolatingTreeMap<Double, Double> shooterLerp = new InterpolatingTreeMap<>();
    InterpolatingTreeMap<Double, Double> hoodLerp = new InterpolatingTreeMap<>();

    public ShooterSetpoint getShot(double distance) {
        //double rpm = shooterInterpolator.interpolate(distance);
        //double hood = hoodInterpolator.interpolate(distance);
        double rpm  = shooterLerp.get(Double.valueOf(distance));
        double hood = hoodLerp.get(Double.valueOf(distance));

        SmartDashboard.putNumber("interpolated shooter rpm", rpm);
        SmartDashboard.putNumber("interpolated hood angle", hood);

        return new ShooterSetpoint(rpm, hood);
    }

    public double getInterpolatedHood(double distance){
        return hoodInterpolator.interpolate(distance);
    }

    public double getInterpolatedShooter(double distance){
        return shooterInterpolator.interpolate(distance);
    }

    public ShooterSetpoint generateArbitraryShot(double rpm, double hood){
        return new ShooterSetpoint(rpm, hood);
    }

    public void initMaps(){
        Double[] distances = new Double[]{
            Double.valueOf(2.3), //1
            Double.valueOf(1), //2
            Double.valueOf(2), //2
            Double.valueOf(3), //3
            Double.valueOf(4), //4
        };

        //distance (meters), speed
        //3000, 10 (2.3)
        //3000, 8 (2.9)
        shooterLerp.put(distances[0], Double.valueOf(3000));
    }
}