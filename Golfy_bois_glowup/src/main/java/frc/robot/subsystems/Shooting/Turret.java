package frc.robot.subsystems.Shooting;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.VisionManager;
import frc.robot.util.CommonConversions;
import frc.robot.util.Limelight;
import frc.robot.util.LiveNumber;

public class Turret extends SubsystemBase{
    
    public enum TurretState{
        OFF,
        JOG,
        SETPOINT,
        DYNAMIC,
        ZERO
    }
    private static Turret instance = new Turret();

    private Rotation2d currentSetpoint = new Rotation2d();
    public double jogVal = 0;

    private Limelight turretLL  = VisionManager.getInstance().getTurretLL();

    TurretState turretState = TurretState.SETPOINT;

    CANSparkMax turretMotor = new CANSparkMax(TurretConstants.DEVICE_ID_TURRET, MotorType.kBrushless);

    PIDController rotationController = new PIDController(TurretConstants.TRACKING_KP, 0, TurretConstants.TRACKING_KD);

    DigitalInput limSwitch = new DigitalInput(TurretConstants.DEVICE_ID_LIMIT_SWITCH);

    private LiveNumber turretP = new LiveNumber("turret P", TurretConstants.TURRET_KP);
    private LiveNumber trackingP = new LiveNumber("tracking P", TurretConstants.TRACKING_KP);
    private LiveNumber trackingD = new LiveNumber("tracking D", TurretConstants.TRACKING_KD);

    public Turret(){
        configMotor();

        rotationController.setTolerance(1);
        rotationController.setSetpoint(0);
    }

    public static Turret getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        
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

            case DYNAMIC:
                trackTarget();
                break;

            case ZERO:
                zeroRoutine();
                break;
        }
    }

    public void logData(){
        SmartDashboard.putNumber("shooter actual position", getAngle().getDegrees());
        SmartDashboard.putNumber("shooter wanted position", currentSetpoint.getDegrees());
    }

    public void goToSetpoint(){
        turretMotor.getPIDController().setReference(
            currentSetpoint.getRotations() * TurretConstants.TURRET_GEARING,
            ControlType.kPosition);
    }

    public void jog(double value){
        turretMotor.set(value);
    }

    public void setSetpoint(Rotation2d wantedSetpoint){
        currentSetpoint = wantedSetpoint;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(turretMotor.getEncoder().getPosition() / TurretConstants.TURRET_GEARING);
    }

    public TurretState getState(){
        return turretState;
    }

    public void setState(TurretState state){
        turretState = state;
    }

    public void zeroRoutine(){
        if(!limSwitch.get()) //if the switch hasnt been hit
            turretMotor.set(0.1);

        else{   //if the switch has been hit
            turretMotor.set(0);
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

        turretMotor.getPIDController().setReference(
            wantedAngle.getRotations() * TurretConstants.TURRET_GEARING,
            ControlType.kPosition);

    }

    public void trackTarget(){
        if(turretLL.hasTargets())
            jog(rotationController.calculate(turretLL.getTargetYaw().getDegrees()));
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
        turretMotor.getPIDController().setP(turretP.get());

        rotationController.setP(trackingP.get());
        rotationController.setD(trackingD.get());
    }

    public void zeroEncoder(){
        turretMotor.getEncoder().setPosition(0);
    }

    public void configMotor(){
        turretMotor.restoreFactoryDefaults();

        turretMotor.setInverted(false);
        turretMotor.setIdleMode(IdleMode.kBrake);
        turretMotor.getEncoder().setPosition(0);

        turretMotor.setSmartCurrentLimit(20, 35);
        turretMotor.enableVoltageCompensation(12);
    
        SparkMaxPIDController controller = turretMotor.getPIDController();

        controller.setP(turretP.get());

    }
    
    
}
