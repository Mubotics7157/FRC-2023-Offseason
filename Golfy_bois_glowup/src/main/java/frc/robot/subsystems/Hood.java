package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.CommonConversions;
import frc.robot.util.Shooting.ShotGenerator;

public class Hood extends SubsystemBase{

    public enum HoodState{
        OFF,
        JOG,
        SETPOINT,
        DYNAMIC,
        ZERO
    }
    
    private static Hood instance = new Hood();

    WPI_TalonFX hoodMotor = new WPI_TalonFX(HoodConstants.DEVICE_ID_HOOD);
    private Rotation2d currentSetpoint = new Rotation2d();
    public double jogVal = 0;
    private HoodState hoodState = HoodState.SETPOINT;

    DigitalInput limSwitch = new DigitalInput(HoodConstants.DEVICE_ID_LIMIT_SWITCH);

    ShotGenerator shotGen = ShotGenerator.getInstance();
    
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

            case DYNAMIC:
                trackTarget();
                //SmartDashboard.putNumber("interpolated hood angle", currentSetpoint.getDegrees());
                break;

            case ZERO:
                zeroRoutine();
                break;
        }
    }

    public void goToSetpoint(){
        hoodMotor.set(ControlMode.Position, CommonConversions.radiansToSteps(currentSetpoint.getDegrees(), HoodConstants.HOOD_GEARING));
    }

    /*
    public void setAngle(Rotation2d angle){
        hoodMotor.set(ControlMode.Position, CommonConversions.radiansToSteps(angle.getDegrees(), HoodConstants.HOOD_GEARING));
    }
    */

    public void setSetpoint(Rotation2d wantedSetpoint){
        currentSetpoint = wantedSetpoint;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(CommonConversions.stepsToRadians(hoodMotor.getSelectedSensorPosition(), HoodConstants.HOOD_GEARING));
    }

    public boolean atSetpoint(){
        return Math.abs(getAngle().getDegrees() - currentSetpoint.getDegrees()) < 2;
    }

    public void trackTarget(){
        if(VisionManager.getInstance().getTurretLL().hasTargets())
            currentSetpoint = shotGen.getInterpolatedHood(VisionManager.getInstance().getDistanceToTarget());
    }

    public void zeroRoutine(){

        if(!limSwitch.get()) //if switch is not hit
            hoodMotor.set(ControlMode.PercentOutput, 0.1);

        else{ //if the switch is hit
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodMotor.setSelectedSensorPosition(0);
        }
    }

    public boolean isZeroed(){
        return limSwitch.get();
    }

    public void setState(HoodState state){
        hoodState = state;
    }

    public HoodState getState(){
        return hoodState;
    }

    
}
