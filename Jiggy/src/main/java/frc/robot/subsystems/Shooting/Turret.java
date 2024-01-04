package frc.robot.subsystems.Shooting;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.VisionManager;
import frc.robot.util.CommonConversions;
import frc.robot.util.Limelight;
import frc.robot.util.LiveNumber;

public class Turret extends SubsystemBase{
    
    public enum TurretState{
        OFF,
        JOG,
        SETPOINT,
        FIELD_ORIENTED,
        DYNAMIC,
        ZERO
    }
    private static Turret instance = new Turret();

    private Rotation2d currentSetpoint = new Rotation2d();
    private Rotation2d fieldOrientedSetpoint = new Rotation2d();
    public double jogVal = 0;

    private Limelight turretLL  = VisionManager.getInstance().getTurretLL();

    TurretState turretState = TurretState.SETPOINT;

    //CANSparkMax turretMotor = new CANSparkMax(TurretConstants.DEVICE_ID_TURRET, MotorType.kBrushless);
    WPI_TalonFX turretMotor = new WPI_TalonFX(TurretConstants.DEVICE_ID_TURRET);

    PIDController rotationController = new PIDController(TurretConstants.TRACKING_KP, 0, TurretConstants.TRACKING_KD);

    DigitalInput limSwitch = new DigitalInput(TurretConstants.DEVICE_ID_LIMIT_SWITCH);

    private LiveNumber turretP = new LiveNumber("turret P", TurretConstants.TURRET_KP);
    private LiveNumber trackingP = new LiveNumber("tracking P", TurretConstants.TRACKING_KP);
    private LiveNumber trackingD = new LiveNumber("tracking D", TurretConstants.TRACKING_KD);

    public Turret(){
        configMotor();

        rotationController.setTolerance(0.2);
        rotationController.setSetpoint(0);
    }

    public static Turret getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        logData();
        
        switch(turretState){
            case OFF:
                jog(0);
                break;

            case JOG:
                jog(jogVal);
                break;

            case SETPOINT:
                goToSetpoint();
                break;

            case FIELD_ORIENTED:
                goToPositionFieldOriented();
                break;

            case DYNAMIC:
                trackTarget();
                break;

            case ZERO:
                zeroRoutine();
                break;
        }
    }

    public void logData(){
        //SmartDashboard.putNumber("shooter actual position", getAngle().getDegrees());
        //SmartDashboard.putNumber("shooter wanted position", currentSetpoint.getDegrees());
        //Logger.getInstance().recordOutput("Turret/State", getState().toString());
        Logger.getInstance().recordOutput("Turret/Tracking Error", rotationController.getPositionError());
        Logger.getInstance().recordOutput("Turret/Position Actual", getAngle().getDegrees());
        Logger.getInstance().recordOutput("Turret/Mag Sensor", limSwitch.get());
    }

    public void goToSetpoint(){
        turretMotor.set(ControlMode.Position, currentSetpoint.getRotations() * 2048 * TurretConstants.TURRET_GEARING);
        Logger.getInstance().recordOutput("Turret/Position Wanted", currentSetpoint.getDegrees());
    }

    public void jog(double value){
        turretMotor.set(value);
    }

    public void setJog(double value){
        jogVal = value;
    }

    public void setSetpoint(Rotation2d wantedSetpoint){
        currentSetpoint = wantedSetpoint;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations((turretMotor.getSelectedSensorPosition() / 2048) / TurretConstants.TURRET_GEARING);
    }

    public void goToPositionFieldOriented(){
        Rotation2d angleToGoTo = fieldOrientedSetpoint.minus(Drive.getInstance().getHeading());

        setSetpoint(angleToGoTo);
        goToSetpoint();
    }

    public TurretState getState(){
        return turretState;
    }

    public void setState(TurretState state){
        turretState = state;
    }

    public void zeroRoutine(){
        if(limSwitch.get() != TurretConstants.MAG_DETECTED){ //if the switch hasnt been hit
            turretMotor.configForwardSoftLimitEnable(false);
            turretMotor.configReverseSoftLimitEnable(false);
            turretMotor.set(0.1);
        }

        else{   //if the switch has been hit
            turretMotor.configForwardSoftLimitEnable(true);
            turretMotor.configReverseSoftLimitEnable(true);
            turretMotor.set(0);
            setState(TurretState.OFF);
            zeroEncoder();
        } 
    }

    public boolean isZeroed(){
        return limSwitch.get();
    }

    public void turretTrack(){
        //prob isnt useful
        Rotation2d wantedAngle = getAngle().plus(turretLL.getTargetYaw());

        if(wantedAngle.getDegrees() > TurretConstants.TURRET_MAX.getDegrees() + 20){
            wantedAngle = Rotation2d.fromDegrees(0);
        }
        else if(wantedAngle.getDegrees() > TurretConstants.TURRET_MAX.getDegrees()){
            wantedAngle = TurretConstants.TURRET_MAX;
        }

        turretMotor.set(ControlMode.Position, wantedAngle.getRotations() * TurretConstants.TURRET_GEARING * 2048);

    }

    public void trackTarget(){
        if(turretLL.hasTargets())
            jog(-rotationController.calculate(turretLL.getTargetYaw().getDegrees()));
        else
            jog(0);
    }

    public boolean atSetpoint(){
        if(turretState == TurretState.DYNAMIC)
            return rotationController.atSetpoint();
        else //(turretState == TurretState.SETPOINT)
            return Math.abs(getAngle().getDegrees() - currentSetpoint.getDegrees()) < 2;
    }

    public void configGains(){
        turretMotor.config_kP(0, turretP.get());

        rotationController.setP(trackingP.get());
        rotationController.setD(trackingD.get());
    }

    public void zeroEncoder(){
        turretMotor.setSelectedSensorPosition(Units.degreesToRotations(140) * 2048 * TurretConstants.TURRET_GEARING);
        //turretMotor.setSelectedSensorPosition(0);
    }

    public void configMotor(){
        turretMotor.configFactoryDefault();

        turretMotor.setInverted(false);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setSelectedSensorPosition(0);

        turretMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0.1));
        turretMotor.configVoltageCompSaturation(12);
        turretMotor.enableVoltageCompensation(true);
       
        turretMotor.configReverseSoftLimitThreshold(-Units.degreesToRotations(125) * 2048 * TurretConstants.TURRET_GEARING);
        turretMotor.configForwardSoftLimitThreshold(Units.degreesToRotations(125) * 2048 * TurretConstants.TURRET_GEARING);

        turretMotor.configForwardSoftLimitEnable(true);
        turretMotor.configReverseSoftLimitEnable(true);

        turretMotor.config_kP(0, TurretConstants.TURRET_KP);

    }
    
    
}
