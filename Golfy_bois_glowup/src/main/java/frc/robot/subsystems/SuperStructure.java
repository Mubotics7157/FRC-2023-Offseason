package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hood.HoodState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Turret.TurretState;

public class SuperStructure extends SubsystemBase{
    
    public enum SuperStructureState{
        AUTO,
        CUSTOM,
        STOW,
        ZERO
    }

    private static SuperStructure instance = new SuperStructure();

    private final Turret turret = Turret.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    private SuperStructureState currentState = SuperStructureState.AUTO;

    public SuperStructure(){

        SmartDashboard.putNumber("Custom hood", 0); //degrees
        SmartDashboard.putNumber("Custom shooter", 0); //degrees
        SmartDashboard.putNumber("Custom turret", 0); //RPM
    }


    public static SuperStructure getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        /* 
        switch(state){
            case AUTO:
                setAll(null, null, null);
                break;
            case CUSTOM:
                setAll(
                    Rotation2d.fromDegrees(SmartDashboard.getNumber("Custom turret", 0)),
                    Rotation2d.fromDegrees(SmartDashboard.getNumber("Custom hood", 0)),
                    SmartDashboard.getNumber("Custom shooter", 0));
                break;
        }

        */

    }



    public void setState(SuperStructureState wantedState){
        currentState = wantedState;

        switch(currentState){
            case STOW:
                setAll(null, null, null);
            break; 
            
            case AUTO:
                setTracking();
                break;

            case CUSTOM:
                setAll(
                    Rotation2d.fromDegrees(SmartDashboard.getNumber("Custom turret", 0)),
                    Rotation2d.fromDegrees(SmartDashboard.getNumber("Custom hood", 0)),
                    SmartDashboard.getNumber("Custom shooter", 0));
                break;

            case ZERO:
                zeroAll();
                break;
        }
    }

    public SuperStructureState getState(){
        return currentState;
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

    public void setAll(Rotation2d turretAng, Rotation2d hoodAng, Double shooterSpeed){
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
