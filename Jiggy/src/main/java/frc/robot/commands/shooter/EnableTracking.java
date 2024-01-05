package frc.robot.commands.shooter;

import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.Shooting.ShooterManager;
import frc.robot.subsystems.Shooting.Turret;
import frc.robot.subsystems.Shooting.ShooterManager.ShooterManagerState;
import frc.robot.subsystems.Shooting.Turret.TurretState;

public class EnableTracking extends CommandBase{
    private ShooterManager shooterManager;
    private Turret turret;

    public EnableTracking(Turret turret){
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        VisionManager.getInstance().setLeds(true);
        turret.setState(TurretState.DYNAMIC);
        //shooterManager.setState(ShooterManagerState.AUTO);
    }

    @Override
    public void end(boolean interrupted) {
        VisionManager.getInstance().setLeds(false);
        turret.setState(TurretState.OFF);
        //shooterManager.setState(ShooterManagerState.STOW);
    }
}
