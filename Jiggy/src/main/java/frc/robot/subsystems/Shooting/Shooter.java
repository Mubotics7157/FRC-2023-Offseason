package frc.robot.subsystems.Shooting;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.VisionManager;
import frc.robot.util.CommonConversions;
import frc.robot.util.LiveNumber;
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

    private CANSparkMax shooterMaster = new CANSparkMax(ShooterConstants.DEVICE_ID_SHOOTER_MASTER, MotorType.kBrushless);
    private CANSparkMax shooterSlave = new CANSparkMax(ShooterConstants.DEVICE_ID_SHOOT_SLAVE, MotorType.kBrushless);

    private double currentSetpoint = 0;

    private double jogValue = 0;


    ShotGenerator shotGen = ShotGenerator.getInstance();

    private LiveNumber shooterP = new LiveNumber("shooter P", ShooterConstants.SHOOTER_KP);
    private LiveNumber shooterF = new LiveNumber("shooter F", ShooterConstants.SHOOTER_KF);

    public Shooter(){
        configMotors();
    }

    public static Shooter getInstance(){
            return instance;
    }

    @Override
    public void periodic() {
        logData();

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

    public void logData(){
        Logger.getInstance().recordOutput("Shooter/RPM", getRPM());
    }

    public void goToSetpoint(){
        shooterMaster.getPIDController().setReference(currentSetpoint * ShooterConstants.SHOOTER_GEARING, ControlType.kVelocity);
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
        shooterMaster.set(value);
    }

    public void setJog(double value){
        jogValue = value;
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
        return shooterMaster.getEncoder().getVelocity();// / ShooterConstants.SHOOTER_GEARING;
    }

    public void configGains(){
        shooterMaster.getPIDController().setP(shooterP.get());
        shooterMaster.getPIDController().setFF(shooterF.get());
    }

    public void configMotors(){
        shooterMaster.restoreFactoryDefaults();
        shooterSlave.restoreFactoryDefaults();
    
        //shooterMaster.setInverted(false);
        //shooterSlave.setInverted(true);

        shooterMaster.setIdleMode(IdleMode.kCoast);
        shooterSlave.setIdleMode(IdleMode.kCoast);

        shooterMaster.setInverted(true);

        shooterSlave.follow(shooterMaster, true);

        //shooterMaster.setSmartCurrentLimit(35);
        shooterMaster.enableVoltageCompensation(12);

        SparkMaxPIDController controller = shooterMaster.getPIDController();
        controller.setP(ShooterConstants.SHOOTER_KP);
        controller.setP(ShooterConstants.SHOOTER_KF);
    }
}
