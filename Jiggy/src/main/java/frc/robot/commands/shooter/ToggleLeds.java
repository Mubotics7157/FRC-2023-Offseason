package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionManager;

public class ToggleLeds extends InstantCommand{
    private VisionManager visionManager;

    public ToggleLeds(VisionManager visionManager){
        this.visionManager = visionManager;

        addRequirements(visionManager);
    }

    @Override
    public void initialize() {
        if(visionManager.getTurretLL().areLedsOn())
            visionManager.getTurretLL().setLEDs(false);
        else
            visionManager.getTurretLL().setLEDs(true);
    }
}
