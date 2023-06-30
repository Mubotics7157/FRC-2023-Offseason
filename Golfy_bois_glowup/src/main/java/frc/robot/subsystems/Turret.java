package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.CommonConversions;
import frc.robot.util.Limelight;

public class Turret extends SubsystemBase{
    
    public enum TurretState{
        OFF,
        JOG,
        SETPOINT,
        AUTO,
        ZERO
    }
    private static Turret instance = new Turret();

    private Rotation2d setpoint = new Rotation2d();
    public double jogVal = 0;

    private Limelight turretLL;

    TurretState turretState = TurretState.SETPOINT;

    WPI_TalonFX turretMotor = new WPI_TalonFX(TurretConstants.DEVICE_ID_TURRET);

    public Turret(){
        turretMotor.configFactoryDefault();

        turretMotor.setInverted(false);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setSelectedSensorPosition(0);

        turretMotor.configPeakOutputForward(1);
        turretMotor.configPeakOutputReverse(-1);

        turretMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 35, 35, 0.1));
        turretMotor.configVoltageCompSaturation(12);
        turretMotor.enableVoltageCompensation(false);
        
        turretMotor.config_kP(0, TurretConstants.TURRET_KP);

        turretLL = VisionManager.getInstance().getTurretLL();
    }

    public static Turret getInstance(){
        if(instance != null)
            return instance;
        else   
            return new Turret();
    }

    @Override
    public void periodic() {
        
        switch(turretState){
            case OFF:
                turretMotor.set(ControlMode.PercentOutput, 0);
                break;
            case JOG:
                turretMotor.set(ControlMode.PercentOutput, jogVal);
                break;
            case SETPOINT:
                goToSetpoint();
                break;
            case AUTO:
                turretTrack();
                break;
            case ZERO:
                break;
        }
    }

    public void goToSetpoint(){
        turretMotor.set(ControlMode.Position, CommonConversions.radiansToSteps(setpoint.getRadians(), TurretConstants.TURRET_GEARING));
    }

    public void setSetpoint(Rotation2d setpoint){
        this.setpoint = setpoint;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(CommonConversions.stepsToRadians(turretMotor.getSelectedSensorPosition(), TurretConstants.TURRET_GEARING));
    }

    public void turretTrack(){
    
        Rotation2d wantedAngle = getAngle().plus(turretLL.getTargetYaw());

        if(wantedAngle.getDegrees() > TurretConstants.TURRET_MAX.getDegrees() + 20){
            wantedAngle = Rotation2d.fromDegrees(0);
        }
        else if(wantedAngle.getDegrees() > TurretConstants.TURRET_MAX.getDegrees()){
            wantedAngle = TurretConstants.TURRET_MAX;
        }

        turretMotor.set(ControlMode.Position, CommonConversions.radiansToSteps(wantedAngle.getRadians(), TurretConstants.TURRET_GEARING));

    }
    
    
}
