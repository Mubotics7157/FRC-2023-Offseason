package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
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
        return instance;
    }

    @Override
    public void periodic() {
        logData();
    }

    public Limelight getTurretLL(){
        return turretLL;
    }

    public boolean hasTargets(){
        return turretLL.hasTargets();
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
        //SmartDashboard.putNumber("limelight Pitch", turretLL.getTargetPitch().getDegrees());
        //SmartDashboard.putNumber("limelight Yaw", turretLL.getTargetYaw().getDegrees());
        Logger.getInstance().recordOutput("Limelight/Calculated Distance", getDistanceToTarget());
        Logger.getInstance().recordOutput("Limelight/Has Targets", turretLL.hasTargets());

        if(hasTargets()){
            Logger.getInstance().recordOutput("Limelight/Target Yaw", turretLL.getTargetYaw().getDegrees());
        }
    }

    public void setLeds(boolean on){
        turretLL.setLEDs(on);
    }

    
}
