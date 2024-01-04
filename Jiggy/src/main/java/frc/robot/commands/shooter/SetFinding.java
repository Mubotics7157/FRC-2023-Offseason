package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.Shooting.Turret;
import frc.robot.subsystems.Shooting.Turret.TurretState;

public class SetFinding extends CommandBase{
    
    private Turret turret;

    public SetFinding(Turret turret){
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setState(TurretState.FINDING);
    }
}
