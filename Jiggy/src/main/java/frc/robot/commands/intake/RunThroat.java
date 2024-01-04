package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intaking.Throat;
import frc.robot.subsystems.Intaking.Throat.ThroatState;

public class RunThroat extends CommandBase{
    private Throat throat;
    private boolean up;

    public RunThroat(Throat throat, boolean up){
        this.throat = throat;
        this.up = up;

        addRequirements(throat);
    }

    @Override
    public void initialize() {
        if(up){
            throat.setState(ThroatState.SHOOTING);
        }
        else
            throat.setState(ThroatState.UNCLOGGING);
    }

    @Override
    public void end(boolean interrupted) {
        throat.setState(ThroatState.OFF);
    }
}
