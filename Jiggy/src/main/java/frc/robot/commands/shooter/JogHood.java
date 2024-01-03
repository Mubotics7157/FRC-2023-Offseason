package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting.Hood;
import frc.robot.subsystems.Shooting.Shooter;
import frc.robot.subsystems.Shooting.Hood.HoodState;
import frc.robot.subsystems.Shooting.Shooter.ShooterState;

public class JogHood extends CommandBase{
    private DoubleSupplier axis;
    private Hood hood;

    public JogHood(DoubleSupplier axis, Hood hood){
        this.axis = axis;
        this.hood = hood;

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setState(HoodState.JOG);
    }

    @Override
    public void execute() {
        if(Math.abs(axis.getAsDouble()) > 0.1)
            hood.setJog(axis.getAsDouble() / 2);
        else
            hood.setJog(0);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setState(HoodState.OFF);
    }
}
