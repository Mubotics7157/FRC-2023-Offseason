package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.CommonConversions;
import frc.robot.util.Shooting.ShotGenerator;

public class Shooter extends SubsystemBase{
    
    public enum ShooterState{
        OFF,
        SETPOINT,
        DYNAMIC,
        JOG
    }

    private static Shooter instance = new Shooter();

    private ShooterState state = ShooterState.SETPOINT;

    WPI_TalonFX shooterMotor = new WPI_TalonFX(ShooterConstants.DEVICE_ID_SHOOTER);

    private double currentSetpoint = 0;

    private double jogValue = 0;

    ShotGenerator shotGen = ShotGenerator.getInstance();

    public Shooter(){

        shooterMotor.configFactoryDefault();
    
        shooterMotor.setInverted(false);
        shooterMotor.setNeutralMode(NeutralMode.Brake);

        shooterMotor.config_kP(0, ShooterConstants.SHOOTER_KP);
        shooterMotor.config_kF(0, ShooterConstants.SHOOTER_KF);

        shooterMotor.configPeakOutputForward(1);
        shooterMotor.configPeakOutputReverse(-1);

        shooterMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 35, 35, 0.1));

        shooterMotor.configVoltageCompSaturation(12);
        shooterMotor.enableVoltageCompensation(false);

    }

    public static Shooter getInstance(){
            return instance;
    }

    @Override
    public void periodic() {

        switch(state){
            case OFF:
                jog(0);
                break;

            case SETPOINT:
                goToSetpoint();
                break;
            
            case JOG:
                jog(jogValue);
                break;

            case DYNAMIC:
                trackTarget();
                //SmartDashboard.putNumber("interpolated shooter rpm", currentSetpoint);
                break;

        }
    }

    public void goToSetpoint(){
        shooterMotor.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(currentSetpoint, ShooterConstants.SHOOTER_GEARING));
    }
    
    public void trackTarget(){
        if(VisionManager.getInstance().getTurretLL().hasTargets())
            currentSetpoint = shotGen.getInterpolatedShooter(VisionManager.getInstance().getDistanceToTarget()); //dynamic values using distance PLACEHOLDER
        
        goToSetpoint();
    }
    public boolean atSpeed(){
        return Math.abs(getRPM() - currentSetpoint) < 50;
    }

    public void jog(double value){
        shooterMotor.set(ControlMode.PercentOutput, value);
    }

    public void setSetpoint(double wantedSetpoint){
        currentSetpoint = wantedSetpoint;
    }

    public void setState(ShooterState wantedState){
        state = wantedState;
    }

    public ShooterState getState(){
        return state;
    }

    public double getRPM(){
        return CommonConversions.stepsPerDecisecToRPM(shooterMotor.getSelectedSensorVelocity(), ShooterConstants.SHOOTER_GEARING);
    }
}
