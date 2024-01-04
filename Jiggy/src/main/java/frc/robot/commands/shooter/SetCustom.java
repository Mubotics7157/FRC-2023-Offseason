package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting.Shooter;
import frc.robot.subsystems.Shooting.ShooterManager;
import frc.robot.subsystems.Shooting.ShooterManager.ShooterManagerState;

public class SetCustom extends CommandBase{
    
    private ShooterManager shooterManager;

    public SetCustom(ShooterManager shooterManager){
        this.shooterManager = shooterManager;

        addRequirements(shooterManager);
    }

    @Override
    public void initialize() {
        shooterManager.setState(ShooterManagerState.CUSTOM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterManager.setState(ShooterManagerState.STOW);
    }
}
