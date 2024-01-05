package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting.Shooter;
import frc.robot.subsystems.Shooting.Turret;
import frc.robot.subsystems.Shooting.Shooter.ShooterState;
import frc.robot.subsystems.Shooting.Turret.TurretState;

public class JogTurret extends CommandBase{
    private DoubleSupplier axis;
    private Turret turret;

    public JogTurret(DoubleSupplier axis, Turret turret){
        this.axis = axis;
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setState(TurretState.JOG);
    }

    @Override
    public void execute() {
        if(Math.abs(axis.getAsDouble()) > 0.1){
            turret.setJog(axis.getAsDouble());
        }
        else 
            turret.setJog(0);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setState(TurretState.OFF);
    }
}
