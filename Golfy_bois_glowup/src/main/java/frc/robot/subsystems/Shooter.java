package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.CommonConversions;

public class Shooter extends SubsystemBase{
    

    private static Shooter instance = new Shooter();

    WPI_TalonFX shooterMotor = new WPI_TalonFX(ShooterConstants.DEVICE_ID_SHOOTER);

    private double setpoint = 0;

    public Shooter(){

        shooterMotor.configFactoryDefault();
    
        shooterMotor.setInverted(false);
        shooterMotor.setNeutralMode(NeutralMode.Brake);

        shooterMotor.config_kP(0, ShooterConstants.SHOOTER_KP);
        shooterMotor.config_kF(0, ShooterConstants.SHOOTER_KF);

        shooterMotor.configPeakOutputForward(1);
        shooterMotor.configPeakOutputReverse(-1);

        shooterMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 35, 35, 0.1));

        shooterMotor.setVoltage(12);
        shooterMotor.enableVoltageCompensation(false);

    }

    public static Shooter getInstance(){
        if(instance != null)
            return instance;
        else
            return new Shooter();
    }

    @Override
    public void periodic() {
        
    }

    public void goToSetpoint(){
        shooterMotor.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(setpoint, ShooterConstants.SHOOTER_GEARING));
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public double getRPM(){
        return CommonConversions.stepsPerDecisecToRPM(shooterMotor.getSelectedSensorVelocity(), ShooterConstants.SHOOTER_GEARING);
    }
}
