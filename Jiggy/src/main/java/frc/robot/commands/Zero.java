package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting.ShooterManager;
import frc.robot.subsystems.Shooting.ShooterManager.ShooterManagerState;

public class Zero extends CommandBase{

    ShooterManager shooterManager;

    ShooterManagerState previousState;
    
    public Zero(ShooterManager shooterManager){
        this.shooterManager = shooterManager;

        addRequirements(shooterManager);
    }
    
    @Override
    public void initialize() {
        shooterManager.setState(ShooterManagerState.ZERO);
        previousState = shooterManager.getState();
    }

    @Override
    public boolean isFinished() {
        return shooterManager.isZeroed();
    }

    @Override
    public void end(boolean interrupted) {
        shooterManager.setState(previousState);
    }
}
