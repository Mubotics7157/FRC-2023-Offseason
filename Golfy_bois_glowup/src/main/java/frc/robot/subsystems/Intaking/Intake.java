package frc.robot.subsystems.Intaking;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LiveNumber;

public class Intake extends SubsystemBase{

    public enum IntakeState{
        STOW,
        DOWN,
        INTAKING,
        OUTTAKING,
        CUSTOM
    }
    
    private static Intake instance = new Intake();

    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.DEVICE_ID_INTAKE, MotorType.kBrushless);
    private CANSparkMax actuatorMotor = new CANSparkMax(IntakeConstants.DEVICE_ID_ACTUATOR, MotorType.kBrushless);

    private IntakeState intakeState = IntakeState.STOW;

    private LiveNumber customSpeed = new LiveNumber("Custom Intake Speed", 0);
    private LiveNumber customActuator = new LiveNumber("Custom Intake Angle", 0);

    private LiveNumber actuatorP = new LiveNumber("Actuator kP", IntakeConstants.ACTUATOR_KP);
    private LiveNumber actuatorD = new LiveNumber("Actuator kD", IntakeConstants.ACTUATOR_KD);

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

            case CUSTOM:
                goToPosition(customActuator.get());
                setIntake(customSpeed.get());
                break;

        }
    }

    private void goToPosition(double position){
        actuatorMotor.getPIDController().setReference(position, ControlType.kPosition);
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

    public void configGains(){
        actuatorMotor.getPIDController().setP(actuatorP.get());
        actuatorMotor.getPIDController().setD(actuatorD.get());
    }

    public void configMotors(){
        intakeMotor.restoreFactoryDefaults();
        actuatorMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(false);
        actuatorMotor.setInverted(false);

        intakeMotor.setIdleMode(IdleMode.kBrake);
        actuatorMotor.setIdleMode(IdleMode.kBrake);

        actuatorMotor.getEncoder().setPosition(0);
        
        SparkMaxPIDController controller = actuatorMotor.getPIDController();
        controller.setP(IntakeConstants.ACTUATOR_KP);
        controller.setD(IntakeConstants.ACTUATOR_KD);
        
    }

}
