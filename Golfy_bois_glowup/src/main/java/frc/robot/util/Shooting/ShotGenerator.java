package frc.robot.util.Shooting;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShotGenerator {

    /* 
    public class ShooterSpeed {

        public final double topSpeed;
        public final double bottomSpeed; 
        public ShooterSpeed(double topSpeed, double bottomSpeed) { 
          this.topSpeed = topSpeed; 
          this.bottomSpeed = bottomSpeed;
        } 
    }
    */

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

    public ShooterSetpoint getShot(double distance) {
        //SplineInterpolator speedInterpolator = wheelInterpolator;
        //SplineInterpolator spinInterpolator = hoodInterpolator;

        double rpm = shooterInterpolator.interpolate(distance);
        double hood = hoodInterpolator.interpolate(distance);
        SmartDashboard.putNumber("interpolated shooter rpm", rpm);
        SmartDashboard.putNumber("interpolated hood angle", hood);
        return new ShooterSetpoint(rpm, hood);
    }

    public Rotation2d getInterpolatedHood(double distance){
        return Rotation2d.fromDegrees(hoodInterpolator.interpolate(distance));
    }

    public double getInterpolatedShooter(double distance){
        return shooterInterpolator.interpolate(distance);
    }

    public ShooterSetpoint generateArbitraryShot(double rpm, double hood){
        return new ShooterSetpoint(rpm, hood);
    }
}