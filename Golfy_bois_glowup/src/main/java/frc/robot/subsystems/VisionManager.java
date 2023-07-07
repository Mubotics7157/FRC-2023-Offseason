package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.Limelight;

public class VisionManager extends SubsystemBase{
    
    public static VisionManager instance = new VisionManager();

    private Limelight turretLL;

    private double lastKnownDistance = 0;

    public VisionManager(){

        turretLL = new Limelight(VisionConstants.TURRET_LL_NAME);
    }

    public static VisionManager getInstance(){
        if(instance != null)
            return instance;
        else
            return new VisionManager();
    }

    @Override
    public void periodic() {
        logData();
    }

    public Limelight getTurretLL(){
        return turretLL;
    }

    public double getDistanceToTarget(){
        if(turretLL.hasTargets()){
            double distance = (VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.TURRET_LL_HEIGHT_METERS) / Math.tan(VisionConstants.TURRET_LL_MOUNTING_PITCH_RADIANS + turretLL.getTargetPitch().getRadians());

            lastKnownDistance = distance;
            return distance;
        }

        else
            return lastKnownDistance;
    }

    public void logData(){
        SmartDashboard.putNumber("limelight Pitch", turretLL.getTargetPitch().getDegrees());
        SmartDashboard.putNumber("limelight Yaw", turretLL.getTargetYaw().getDegrees());
    }
}
