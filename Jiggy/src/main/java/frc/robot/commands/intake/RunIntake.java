package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intaking.IntakeManager;
import frc.robot.subsystems.Intaking.IntakeManager.IntakeManagerState;

public class RunIntake extends CommandBase{

    private IntakeManager intakeManager;

    public RunIntake(IntakeManager intakeManager){
        this.intakeManager = intakeManager;

        addRequirements(intakeManager);
    }

    @Override
    public void initialize() {
        intakeManager.setState(IntakeManagerState.INTAKING);
    }

    @Override
    public void end(boolean interrupted) {
        intakeManager.setState(IntakeManagerState.STOW);
    }
}
