package frc.robot.subsystems.Shooting;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.VisionManager;
import frc.robot.util.CommonConversions;
import frc.robot.util.LiveNumber;
import frc.robot.util.Shooting.ShotGenerator;

public class Hood extends SubsystemBase{

    public enum HoodState{
        OFF,
        JOG,
        SETPOINT,
        DYNAMIC,
        STOW,
        ZERO
    }
    
    private static Hood instance = new Hood();

    private CANSparkMax hoodMotor = new CANSparkMax(HoodConstants.DEVICE_ID_HOOD, MotorType.kBrushless);
    private double currentSetpoint = 0;
    public double jogVal = 0;
    private HoodState hoodState = HoodState.SETPOINT;

    DigitalInput limSwitch = new DigitalInput(HoodConstants.DEVICE_ID_LIMIT_SWITCH);

    private LiveNumber hoodP = new LiveNumber("hood P", HoodConstants.HOOD_KP);

    ShotGenerator shotGen = ShotGenerator.getInstance();
    
    public Hood(){

        configMotor();
    }

    public static Hood getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        logData();
        
        switch(hoodState){
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
                //SmartDashboard.putNumber("interpolated hood angle", currentSetpoint.getDegrees());
                break;

            case STOW:
                stow();
                break;

            case ZERO:
                zeroRoutine();
                break;
        }
    }

    private void logData(){
        Logger.getInstance().recordOutput("Hood/Position Actual", getAngle());
        Logger.getInstance().recordOutput("Hood/Mag sensor", limSwitch.get());
        Logger.getInstance().recordOutput("Hood/State", getState().toString());
    }

    private void goToSetpoint(){
        hoodMotor.getPIDController().setReference(currentSetpoint, ControlType.kPosition);
        Logger.getInstance().recordOutput("Hood/Position Wanted", currentSetpoint);
    }

    public void jog(double value){
        hoodMotor.set(value);
    }

    public void setJog(double value){
        jogVal = value;
    }

    public void setSetpoint(double wantedSetpoint){
        currentSetpoint = wantedSetpoint;
    }

    public double getAngle(){
        return hoodMotor.getEncoder().getPosition();
    }

    public boolean atSetpoint(){
        return Math.abs(getAngle() - currentSetpoint) < 2;
    }

    public void stow(){
        if(limSwitch.get() != HoodConstants.MAG_DETECTED){ //if switch is not hit
            hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
            hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
            jog(-0.1);
        }

        else{ //if the switch is hit
            hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
            hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
            jog(0);
            hoodMotor.getEncoder().setPosition(0);
        }
    }

    public void trackTarget(){
        if(VisionManager.getInstance().getTurretLL().hasTargets())
            currentSetpoint = shotGen.getInterpolatedHood(VisionManager.getInstance().getDistanceToTarget());

        goToSetpoint();
    }

    public void zeroRoutine(){
        if(limSwitch.get() != HoodConstants.MAG_DETECTED){ //if switch is not hit
            hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
            hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
            jog(-0.1);
        }

        else{ //if the switch is hit
            hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
            hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
            jog(0);
            hoodMotor.getEncoder().setPosition(0);
            setState(HoodState.OFF);
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

    public void configGains(){
        hoodMotor.getPIDController().setP(hoodP.get());
    }

    public void configMotor(){
        hoodMotor.restoreFactoryDefaults();

        hoodMotor.setInverted(false);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        hoodMotor.getEncoder().setPosition(0);

        hoodMotor.setSmartCurrentLimit(20, 35);
        hoodMotor.enableVoltageCompensation(12);

        hoodMotor.setSoftLimit(SoftLimitDirection.kForward, (float)10.5);
        hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.5);

        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        hoodMotor.getPIDController().setP(HoodConstants.HOOD_KP);

        hoodMotor.burnFlash();
    }
    
}
