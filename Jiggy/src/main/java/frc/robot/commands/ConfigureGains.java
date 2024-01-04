package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intaking.IntakeManager;
import frc.robot.subsystems.Shooting.ShooterManager;

public class ConfigureGains extends CommandBase{
    
    private ShooterManager shooterManager;
    private IntakeManager intakeManager;

    public ConfigureGains(ShooterManager shooterManager, IntakeManager intakeManager){
        this.shooterManager = shooterManager;
        this.intakeManager = intakeManager;

        addRequirements(shooterManager, intakeManager);
    }

    @Override
    public void initialize() {
        shooterManager.configGains();
        intakeManager.configGains();
    }
}
