package frc.robot.subsystems.Intaking;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.util.CommonConversions;
import frc.robot.util.LiveNumber;

public class Spindexer extends SubsystemBase{

    public enum SpindexerState{
        OFF,
        JOG,
        IDLE,
        INTAKING,
        SHOOTING,
        CUSTOM
    }
    
    private static Spindexer instance = new Spindexer();

    private WPI_TalonFX spindexerMotor = new WPI_TalonFX(SpindexerConstants.DEVICE_ID_SPINDEXER);

    private SpindexerState spindexerState = SpindexerState.OFF;

    private double jogValue = 0;

    //private LiveNumber spindexerP = new LiveNumber("spindexer P", SpindexerConstants.SPINDEXER_KP);
    //private LiveNumber spindexerD = new LiveNumber("spindexer D", SpindexerConstants.SPINDEXER_KD);
    //private LiveNumber spindexerF = new LiveNumber("spindexer F", SpindexerConstants.SPINDEXER_KF);

    private LiveNumber rampRate = new LiveNumber("spindexer Ramp Rate", SpindexerConstants.SPINDEXER_RAMP_RATE);

    private double invert = 1;

    private LiveNumber customSpeed = new LiveNumber("spindexer speed", 0);

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
                spin(0);
                break;

            case JOG:
                spin(jogValue);
                break;
            
            case IDLE:
                spin(0.2);
                break;
            
            case INTAKING:
                //spin(SpindexerConstants.INTAKING_SPEED);
                spin(0.15);
                break;
            
            case SHOOTING:
                spin(0.2);
                break;

            case CUSTOM:
                spin(customSpeed.get());
                break;
        }
    }

    public void setJog(double value){
        jogValue = value;
    }

    public void spin(double value){
        spindexerMotor.set(ControlMode.PercentOutput, invert * value);
    }

    public void setState(SpindexerState newState){
        if(spindexerState != newState)
            spindexerState = newState;
    }

    public void setInverted(boolean isInverted){
        invert = isInverted ? 1 : -1;
    }

    public SpindexerState getState(){
        return spindexerState;
    }

    public void logData(){
        Logger.getInstance().recordOutput("Spindexer/Amps", spindexerMotor.getStatorCurrent());
    }

    public void configGains(){
        //spindexerMotor.config_kP(0, spindexerP.get());
        //spindexerMotor.config_kD(0, spindexerD.get());
        //spindexerMotor.config_kF(0, spindexerF.get());

        //spindexerMotor.configOpenloopRamp(rampRate.get());
    }

    public void configMotors(){
        spindexerMotor.configFactoryDefault();

        spindexerMotor.setInverted(true);

        spindexerMotor.setNeutralMode(NeutralMode.Coast);

        spindexerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);

        spindexerMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 20, 0.1));

        //spindexerMotor.configOpenloopRamp(SpindexerConstants.SPINDEXER_RAMP_RATE);

        //spindexerMotor.config_kP(0, SpindexerConstants.SPINDEXER_KP);
        //spindexerMotor.config_kD(0, SpindexerConstants.SPINDEXER_KD);
        //spindexerMotor.config_kF(0, SpindexerConstants.SPINDEXER_KF);
    }
}
