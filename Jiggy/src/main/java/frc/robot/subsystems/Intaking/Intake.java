package frc.robot.subsystems.Intaking;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
        OFF,
        STOW,
        DOWN,
        INTAKING,
        OUTTAKING,
        CUSTOM
    }
    
    private static Intake instance = new Intake();

    //private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.DEVICE_ID_INTAKE, MotorType.kBrushless);
    private WPI_TalonFX intakeMotor = new WPI_TalonFX(IntakeConstants.DEVICE_ID_INTAKE);
    private WPI_TalonFX actuatorMotor = new WPI_TalonFX(IntakeConstants.DEVICE_ID_ACTUATOR);
    //private CANSparkMax actuatorMotor = new CANSparkMax(IntakeConstants.DEVICE_ID_ACTUATOR, MotorType.kBrushless);

    private IntakeState intakeState = IntakeState.STOW;

    private LiveNumber customSpeed = new LiveNumber("Custom Intake Speed", 0);
    private LiveNumber customActuator = new LiveNumber("Custom Intake Angle", 0);

    private LiveNumber actuatorP = new LiveNumber("Actuator kP", IntakeConstants.ACTUATOR_KP);
    private LiveNumber actuatorD = new LiveNumber("Actuator kD", IntakeConstants.ACTUATOR_KD);

    private LiveNumber intakeP = new LiveNumber("Intake kP", IntakeConstants.INTAKE_KP);
    private LiveNumber intakeFF = new LiveNumber("Intake kF", IntakeConstants.INTAKE_KF);

    public Intake(){
        configMotors();
    }

    public static Intake getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        logData();
        
        switch(intakeState){
            case OFF:
                jogActuator(0);
                jogIntake(0);
                break;
                
            case STOW:
                goToPosition(IntakeConstants.ACTUATOR_STOW);
                jogIntake(0);
                break;

            case DOWN:
                goToPosition(IntakeConstants.ACTUATOR_DOWN);
                jogIntake(0);
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
        //actuatorMotor.getPIDController().setReference(position, ControlType.kPosition);
        actuatorMotor.set(ControlMode.Position, position);
    }

    private void jogIntake(double value){
        intakeMotor.set(value);
    }

    private void jogActuator(double value){
        actuatorMotor.set(value);
    }

    private void setIntake(double rpm){
        intakeMotor.set(ControlMode.Velocity, rpm);
    }

    public void setState(IntakeState newState){
        if(intakeState != newState)
            intakeState = newState;
    }

    public IntakeState getState(){
        return intakeState;
    }

    public void zeroEncoder(){
        actuatorMotor.setSelectedSensorPosition(0);
    }

    public void logData(){
        Logger.getInstance().recordOutput("Intake/Actuator Position", getPosition());
        Logger.getInstance().recordOutput("Intake/Intake Speed", getRPM());
    }

    public double getPosition(){
        return actuatorMotor.getSelectedSensorPosition();
    }

    public double getRPM(){
        return intakeMotor.getSelectedSensorVelocity();
    }

    public void configGains(){
        //actuatorMotor.getPIDController().setP(actuatorP.get());
        //actuatorMotor.getPIDController().setD(actuatorD.get());
        actuatorMotor.config_kP(0, actuatorP.get());
        actuatorMotor.config_kD(0, actuatorD.get());

        intakeMotor.config_kP(0, intakeP.get());
        intakeMotor.config_kF(0, intakeFF.get());
    }

    public void configMotors(){
        intakeMotor.configFactoryDefault();
        actuatorMotor.configFactoryDefault();

        intakeMotor.setInverted(true);
        actuatorMotor.setInverted(false);

        intakeMotor.setNeutralMode(NeutralMode.Brake);
        actuatorMotor.setNeutralMode(NeutralMode.Brake);

        //intakeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 35, 0.1));

        actuatorMotor.setSelectedSensorPosition(0);
        actuatorMotor.configAllowableClosedloopError(0, 0);

        intakeMotor.config_kP(0, IntakeConstants.INTAKE_KP);
        intakeMotor.config_kF(0, IntakeConstants.INTAKE_KF);

        actuatorMotor.config_kP(0, IntakeConstants.ACTUATOR_KP);
        actuatorMotor.config_kD(0, IntakeConstants.ACTUATOR_KD);
        
    }

}
