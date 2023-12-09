package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.util.CommonConversions;

public class Spindexer extends SubsystemBase{

    public enum SpindexerState{
        OFF,
        IDLE,
        INTAKING,
        SHOOTING
    }
    
    private static Spindexer instance = new Spindexer();

    private WPI_TalonFX spindexerMotor = new WPI_TalonFX(SpindexerConstants.DEVICE_ID_SPINDEXER);

    private SpindexerState spindexerState = SpindexerState.OFF;

    private double throatSetpoint = 0;

    public Spindexer(){
        configMotors();
    }

    public static Spindexer getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        
        switch(spindexerState){
            case OFF:
                jogSpindexer(0);
                break;
            
            case IDLE:
                spin(SpindexerConstants.IDLE_SPEED);
                break;
            
            case INTAKING:
                spin(SpindexerConstants.INTAKING_SPEED);
                break;
            
            case SHOOTING:
                spin(SpindexerConstants.SHOOTING_SPEED);
                break;
        }
    }

    public void jogSpindexer(double value){
        spindexerMotor.set(ControlMode.PercentOutput, value);
    }

    public void spin(double value){
        spindexerMotor.set(ControlMode.MotionMagic, value);
    }

    public void setState(SpindexerState newState){
        if(spindexerState != newState)
            spindexerState = newState;
    }

    public SpindexerState getState(){
        return spindexerState;
    }

    public void configMotors(){
        spindexerMotor.configFactoryDefault();

        spindexerMotor.setInverted(false);

        spindexerMotor.setNeutralMode(NeutralMode.Coast);

        spindexerMotor.configMotionAcceleration(SpindexerConstants.MAX_ACCELERATION * SpindexerConstants.SPINDEXER_GEARING);
        spindexerMotor.configMotionCruiseVelocity(SpindexerConstants.MAX_VELOCITY * SpindexerConstants.SPINDEXER_GEARING);

        spindexerMotor.config_kP(0, SpindexerConstants.SPINDEXER_KP);
        spindexerMotor.config_kD(0, SpindexerConstants.SPINDEXER_KD);
        spindexerMotor.config_kF(0, SpindexerConstants.SPINDEXER_KF);
    }
}
