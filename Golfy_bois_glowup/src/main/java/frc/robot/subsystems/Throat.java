package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ThroatConstants;

public class Throat extends SubsystemBase{
    
    public enum ThroatState{
        OFF,
        SHOOTING,
        UNCLOGGING
    }
    
    private static Throat instance = new Throat();

    private CANSparkMax throatMotor = new CANSparkMax(ThroatConstants.DEVICE_ID_THROAT, MotorType.kBrushless);

    private ThroatState throatState = ThroatState.OFF;

    public Throat(){
        configMotor();
    }

    public static Throat getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        switch(throatState){
            case OFF:
                jogThroat(0);
                break;
            
            case SHOOTING:
                jogThroat(ThroatConstants.THROAT_SHOOTING_SPEED);
                break;
            
            case UNCLOGGING:
                jogThroat(ThroatConstants.THROAT_UNCLOG_SPEED);
                break;
        }
    }

    public void setState(ThroatState newState){
        if(throatState != newState)
            throatState = newState;
    }

    public ThroatState getState(){
        return throatState;
    }
    
    public void jogThroat(double value){
        throatMotor.set(value);
    }
    

    public void configMotor(){
        throatMotor.restoreFactoryDefaults();
        
        throatMotor.setInverted(false);

        throatMotor.setIdleMode(IdleMode.kBrake);
    }
}
