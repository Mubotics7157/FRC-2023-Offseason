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
        ZERO
    }

    private static SuperStructure instance = new SuperStructure();

    private Turret turret = Turret.getInstance();
    private Hood hood = Hood.getInstance();
    private Shooter shooter = Shooter.getInstance();

    private SuperStructureState currentState = SuperStructureState.AUTO;

    public SuperStructure(){

        SmartDashboard.putNumber("Custom hood", 0); //degrees
        SmartDashboard.putNumber("Custom shooter", 0); //degrees
        SmartDashboard.putNumber("Custom turret", 0); //RPM
    }


    public static SuperStructure getInstance(){
        if(instance != null)
            return instance;
        else
            return new SuperStructure();
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

    public void setTracking(){
        if(turret.getState() != TurretState.DYNAMIC)
            turret.setState(TurretState.DYNAMIC);
        
        if(hood.getState() != HoodState.DYNAMIC)
            hood.setState(HoodState.DYNAMIC);

        if(shooter.getState() != ShooterState.DYNAMIC)
            shooter.setState(ShooterState.DYNAMIC);
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
