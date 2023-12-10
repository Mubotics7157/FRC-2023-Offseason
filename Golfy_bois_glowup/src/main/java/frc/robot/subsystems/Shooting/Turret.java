package frc.robot.subsystems.Shooting;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.VisionManager;
import frc.robot.util.CommonConversions;
import frc.robot.util.Limelight;

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

    private Limelight turretLL;

    TurretState turretState = TurretState.SETPOINT;

    WPI_TalonFX turretMotor = new WPI_TalonFX(TurretConstants.DEVICE_ID_TURRET);

    PIDController rotationController = new PIDController(TurretConstants.TRACKING_KP, TurretConstants.TRACKING_KI, TurretConstants.TRACKING_KD);

    SlewRateLimiter speedLimiter = new SlewRateLimiter(90, 90, 0);
    DigitalInput limSwitch = new DigitalInput(TurretConstants.DEVICE_ID_LIMIT_SWITCH);

    public Turret(){
        turretMotor.configFactoryDefault();

        turretMotor.setInverted(false);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setSelectedSensorPosition(0);

        turretMotor.configPeakOutputForward(1);
        turretMotor.configPeakOutputReverse(-1);

        turretMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 35, 35, 0.1));
        turretMotor.configVoltageCompSaturation(12);
        turretMotor.enableVoltageCompensation(false);
        
        turretMotor.config_kP(0, TurretConstants.TURRET_KP);

        rotationController.setTolerance(1);
        rotationController.setSetpoint(0);


        turretLL = VisionManager.getInstance().getTurretLL();
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

    public void goToSetpoint(){
        turretMotor.set(ControlMode.Position, CommonConversions.radiansToSteps(currentSetpoint.getRadians(), TurretConstants.TURRET_GEARING));
    }

    public void jog(double value){
        turretMotor.set(ControlMode.PercentOutput, value);
    }

    public void setSetpoint(Rotation2d wantedSetpoint){


        currentSetpoint = wantedSetpoint;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(CommonConversions.stepsToRadians(turretMotor.getSelectedSensorPosition(), TurretConstants.TURRET_GEARING));
    }

    public TurretState getState(){
        return turretState;
    }

    public void setState(TurretState state){
        turretState = state;
    }

    public void zeroRoutine(){
        if(!limSwitch.get()) //if the switch hasnt been hit
            turretMotor.set(ControlMode.PercentOutput, 0.1);

        else{   //if the switch has been hit
            turretMotor.set(ControlMode.PercentOutput, 0);
            turretMotor.setSelectedSensorPosition(0);
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

        turretMotor.set(ControlMode.Position, CommonConversions.radiansToSteps(wantedAngle.getRadians(), TurretConstants.TURRET_GEARING));

    }

    public void trackTarget(){
        if(turretLL.hasTargets())
            turretMotor.setVoltage(rotationController.calculate(turretLL.getTargetYaw().getDegrees()));
    }

    public boolean atSetpoint(){
        if(turretState == TurretState.DYNAMIC)
            return rotationController.atSetpoint();
        else //(turretState == TurretState.SETPOINT)
            return Math.abs(getAngle().getDegrees() - currentSetpoint.getDegrees()) < 2;
    }
    
    
}
