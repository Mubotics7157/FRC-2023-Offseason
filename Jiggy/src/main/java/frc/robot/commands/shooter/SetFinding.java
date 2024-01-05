package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.Shooting.Turret;
import frc.robot.subsystems.Shooting.Turret.TurretState;

public class SetFinding extends InstantCommand{
    
    private Turret turret;

    public SetFinding(Turret turret){
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        VisionManager.getInstance().setLeds(true);
        turret.setState(TurretState.FINDING);
    }
}
