package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting.Turret;
import frc.robot.subsystems.Shooting.Turret.TurretState;
import frc.robot.util.LiveNumber;

public class SetFieldOriented extends CommandBase{
    
    private Turret turret;

    private LiveNumber setpoint = new LiveNumber("Field Oriented Setpoint", 0);

    public SetFieldOriented(Turret turret){
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.SetFieldOrientedSetpoint(Rotation2d.fromDegrees(setpoint.get()));

        turret.setState(TurretState.FIELD_ORIENTED);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setState(TurretState.OFF);
    }
}
