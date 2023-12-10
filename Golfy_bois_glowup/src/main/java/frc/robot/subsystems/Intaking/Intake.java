package frc.robot.subsystems.Intaking;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    public enum IntakeState{
        STOW,
        DOWN,
        INTAKING,
        OUTTAKING
    }
    
    private static Intake instance = new Intake();

    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.DEVICE_ID_INTAKE, MotorType.kBrushless);
    private CANSparkMax actuationMotor = new CANSparkMax(IntakeConstants.DEVICE_ID_ACTUATOR, MotorType.kBrushless);

    private IntakeState intakeState = IntakeState.STOW;

    public Intake(){
        configMotors();
    }

    public static Intake getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        
        switch(intakeState){
            case STOW:
                goToPosition(IntakeConstants.ACTUATOR_STOW);
                setIntake(0);
                break;

            case DOWN:
                goToPosition(IntakeConstants.ACTUATOR_DOWN);
                setIntake(0);
                break;

            case INTAKING:
                goToPosition(IntakeConstants.ACTUATOR_DOWN);
                setIntake(IntakeConstants.INTAKE_SPEED);
                break;
            
            case OUTTAKING:
                goToPosition(IntakeConstants.ACTUATOR_DOWN);
                setIntake(IntakeConstants.OUTTAKE_SPEED);
                break;
        }
    }

    private void goToPosition(double position){
        actuationMotor.getPIDController().setReference(position, ControlType.kPosition);
    }

    private void setIntake(double value){
        intakeMotor.set(value);
    }

    public void setState(IntakeState newState){
        if(intakeState != newState)
            intakeState = newState;
    }

    public IntakeState getState(){
        return intakeState;
    }

    public void configMotors(){
        intakeMotor.restoreFactoryDefaults();
        actuationMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(false);
        actuationMotor.setInverted(false);

        intakeMotor.setIdleMode(IdleMode.kBrake);
        actuationMotor.setIdleMode(IdleMode.kBrake);

        actuationMotor.getEncoder().setPosition(0);
        
        SparkMaxPIDController controller = actuationMotor.getPIDController();
        controller.setP(IntakeConstants.ACTUATOR_KP);
        controller.setD(IntakeConstants.ACTUATOR_KD);
        
    }

}
