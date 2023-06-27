package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private SuperStructureState state = SuperStructureState.AUTO;

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

    }



    public void setState(SuperStructureState state){
        this.state = state;
    }

    public void setAll(Rotation2d turretAng, Rotation2d hoodAng, Double shooterSpeed){
        turret.setSetpoint(turretAng);
        hood.setSetpoint(hoodAng);
        shooter.setSetpoint(shooterSpeed);
    }
}
