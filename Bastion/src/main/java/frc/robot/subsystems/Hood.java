package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.CommonConversions;

public class Hood extends SubsystemBase{

    public enum HoodState{
        OFF,
        JOG,
        SETPOINT,
        ZERO
    }
    
    private static Hood instance = new Hood();

    WPI_TalonFX hoodMotor = new WPI_TalonFX(HoodConstants.DEVICE_ID_HOOD);
    private Rotation2d setpoint = new Rotation2d();
    public double jogVal = 0;
    private HoodState hoodState = HoodState.SETPOINT;
    
    public Hood(){

        hoodMotor.configFactoryDefault();

        hoodMotor.setInverted(false);
        hoodMotor.setNeutralMode(NeutralMode.Brake);
        hoodMotor.setSelectedSensorPosition(0);

        hoodMotor.configPeakOutputForward(1);
        hoodMotor.configPeakOutputReverse(-1);

        hoodMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 35, 35, 0.1));
        hoodMotor.configVoltageCompSaturation(12);
        hoodMotor.enableVoltageCompensation(false);
        
        hoodMotor.config_kP(0, HoodConstants.HOOD_KP);



    }

    public static Hood getInstance(){
        if(instance != null)
            return instance;
        
        else
            return new Hood();
    }

    @Override
    public void periodic() {
        
        switch(hoodState){
            case OFF:
                hoodMotor.set(ControlMode.PercentOutput, 0);
                break;
            case JOG:
                hoodMotor.set(ControlMode.PercentOutput, jogVal);
                break;
            case SETPOINT:
                goToSetpoint();
                break;
            case ZERO:
                zeroRoutine();
                break;
        }
    }

    public void goToSetpoint(){
        hoodMotor.set(ControlMode.Position, CommonConversions.radiansToSteps(setpoint.getDegrees(), HoodConstants.HOOD_GEARING));
    }

    /*
    public void setAngle(Rotation2d angle){
        hoodMotor.set(ControlMode.Position, CommonConversions.radiansToSteps(angle.getDegrees(), HoodConstants.HOOD_GEARING));
    }
    */

    public void setSetpoint(Rotation2d setpoint){
        this.setpoint = setpoint;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(CommonConversions.stepsToRadians(hoodMotor.getSelectedSensorPosition(), HoodConstants.HOOD_GEARING));
    }
    public void zeroRoutine(){

    }

    public void setState(HoodState state){
        hoodState = state;
    }

    
}
