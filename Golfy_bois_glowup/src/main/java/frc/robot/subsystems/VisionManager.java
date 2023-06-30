package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.Limelight;

public class VisionManager extends SubsystemBase{
    
    public static VisionManager instance = new VisionManager();

    private Limelight turretLL;

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

    public void logData(){
        SmartDashboard.putNumber("limelight Pitch", turretLL.getTargetPitch().getDegrees());
        SmartDashboard.putNumber("limelight Yaw", turretLL.getTargetYaw().getDegrees());
    }
}
