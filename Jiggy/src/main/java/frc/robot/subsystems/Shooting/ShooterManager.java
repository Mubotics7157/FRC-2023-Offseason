package frc.robot.subsystems.Shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Setpoints;
import frc.robot.subsystems.Shooting.Hood.HoodState;
import frc.robot.subsystems.Shooting.Shooter.ShooterState;
import frc.robot.subsystems.Shooting.Turret.TurretState;
import frc.robot.util.LiveNumber;

public class ShooterManager extends SubsystemBase{
    
    public enum ShooterManagerState{
        OFF,
        AUTO,
        CUSTOM,
        SETPOINT,
        STOW,
        ZERO
    }

    private static ShooterManager instance = new ShooterManager();

    private final Turret turret = Turret.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    private ShooterManagerState currentState = ShooterManagerState.AUTO;

    private LiveNumber customHood = new LiveNumber("Custom Hood", 0); //not degrees
    private LiveNumber customShooter = new LiveNumber("Custom Shooter", 0); //rpm
    private LiveNumber customTurret = new LiveNumber("Custom Turret", 0); //degrees

    public ShooterManager(){

    }


    public static ShooterManager getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
    
    }


    public void setState(ShooterManagerState wantedState){
        currentState = wantedState;

        switch(currentState){
            case OFF:
                setStates(TurretState.OFF, HoodState.OFF, ShooterState.OFF);
                break;

            case STOW:
                //setAll(Setpoints.TURRET_STOW, Setpoints.HOOD_STOW, 0);
                hood.setState(HoodState.STOW);
                shooter.setState(ShooterState.OFF);
                break; 
            
            case AUTO:
                setTracking();
                break;

            case SETPOINT:
                setStates(TurretState.SETPOINT, HoodState.SETPOINT, ShooterState.SETPOINT);
                break;
            

            case CUSTOM:
            /* 
                setAll(
                    Rotation2d.fromDegrees(customTurret.get()),
                    customHood.get(),
                    customShooter.get());
                    */

                shooter.setState(ShooterState.SETPOINT);
                shooter.setSetpoint(customShooter.get());

                hood.setState(HoodState.SETPOINT);
                hood.setSetpoint(customHood.get());
                break;

            case ZERO:
                zeroAll();
                break;
        }
    }

    public ShooterManagerState getState(){
        return currentState;
    }

    public void configGains(){
        turret.configGains();
        hood.configGains();
        shooter.configGains();
    }

    public void zeroAll(){
        turret.setState(TurretState.ZERO);
        hood.setState(HoodState.ZERO);
    }

    public boolean readyToShoot(){
        return shooter.atSpeed() && hood.atSetpoint() && turret.atSetpoint();
    }

    public boolean isZeroed(){
        return turret.isZeroed() && hood.isZeroed();
    }

    public void setStates(TurretState turretState, HoodState hoodState, ShooterState shooterState){
        if(turret.getState() != turretState)
            turret.setState(turretState);
        
        if(hood.getState() != hoodState)
            hood.setState(hoodState);

        if(shooter.getState() != shooterState)
            shooter.setState(shooterState);
    }

    public void setTracking(){
        setStates(
            TurretState.DYNAMIC,
            HoodState.DYNAMIC,
            ShooterState.DYNAMIC
        );
    }

    public void setAll(Rotation2d turretAng, double hoodAng, double shooterSpeed){
        if(turret.getState() != TurretState.SETPOINT)
            turret.setState(TurretState.SETPOINT);

        if(hood.getState() != HoodState.SETPOINT)
            hood.setState(HoodState.SETPOINT);

        if(shooter.getState() != ShooterState.SETPOINT)
            shooter.setState(ShooterState.SETPOINT);
            
        turret.setSetpoint(turretAng);
        hood.setSetpoint(hoodAng);
        shooter.setSetpoint(shooterSpeed);
    }
}
