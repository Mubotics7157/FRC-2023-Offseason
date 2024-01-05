package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting.ShooterManager;
import frc.robot.subsystems.Shooting.ShooterManager.ShooterManagerState;

public class Zero extends CommandBase{
    
    private ShooterManager shooterManager;

    public Zero(ShooterManager shooterManager){
        this.shooterManager = shooterManager;

        addRequirements(shooterManager);
    }

    @Override
    public void initialize() {
        shooterManager.setState(ShooterManagerState.ZERO);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
