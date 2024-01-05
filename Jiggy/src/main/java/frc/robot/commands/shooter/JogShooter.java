package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting.Shooter;
import frc.robot.subsystems.Shooting.Shooter.ShooterState;

public class JogShooter extends CommandBase{
    private DoubleSupplier axis;
    private Shooter shooter;

    public JogShooter(DoubleSupplier axis, Shooter shooter){
        this.axis = axis;
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setState(ShooterState.JOG);
    }

    @Override
    public void execute() {
        shooter.setJog(axis.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setState(ShooterState.OFF);
    }
}
