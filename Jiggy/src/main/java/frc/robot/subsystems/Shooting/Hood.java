package frc.robot.subsystems.Shooting;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
        ZERO
    }
    
    private static Hood instance = new Hood();

    private CANSparkMax hoodMotor = new CANSparkMax(HoodConstants.DEVICE_ID_HOOD, MotorType.kBrushless);
    private Rotation2d currentSetpoint = new Rotation2d();
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

            case ZERO:
                zeroRoutine();
                break;
        }
    }

    private void logData(){
        Logger.getInstance().recordOutput("Hood/Position", getAngle().getDegrees());
        Logger.getInstance().recordOutput("Hood/At Setpoint", atSetpoint());
    }

    private void goToSetpoint(){
        hoodMotor.getPIDController().setReference(currentSetpoint.getRotations() * HoodConstants.HOOD_GEARING, ControlType.kPosition);
    }

    public void jog(double value){
        hoodMotor.set(value);
    }

    public void setSetpoint(Rotation2d wantedSetpoint){
        currentSetpoint = wantedSetpoint;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(hoodMotor.getEncoder().getPosition() * HoodConstants.HOOD_GEARING);
    }

    public boolean atSetpoint(){
        return Math.abs(getAngle().getDegrees() - currentSetpoint.getDegrees()) < 2;
    }

    public void trackTarget(){
        if(VisionManager.getInstance().getTurretLL().hasTargets())
            currentSetpoint = shotGen.getInterpolatedHood(VisionManager.getInstance().getDistanceToTarget());

        goToSetpoint();
    }

    public void zeroRoutine(){

        if(!limSwitch.get()) //if switch is not hit
            jog(0.1);

        else{ //if the switch is hit
            jog(0);
            hoodMotor.getEncoder().setPosition(0);
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
        
        hoodMotor.getPIDController().setP(HoodConstants.HOOD_KP);
    }
    
}