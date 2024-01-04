package frc.robot.commands.intake;

import edu.wpi.first.hal.simulation.SPIDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intaking.IntakeManager;
import frc.robot.subsystems.Intaking.Spindexer;
import frc.robot.subsystems.Intaking.IntakeManager.IntakeManagerState;
import frc.robot.subsystems.Intaking.Spindexer.SpindexerState;

public class RunSpindexer extends CommandBase{
    private IntakeManager intakeManager;
    private boolean turn;

    public RunSpindexer(IntakeManager intakeManager, boolean turn){
        this.intakeManager = intakeManager;
        this.turn = turn;

        addRequirements(intakeManager);
    }

    @Override
    public void initialize() {
        Spindexer.getInstance().setInverted(turn);
        intakeManager.setState(IntakeManagerState.SHOOTING);
    }

    @Override
    public void end(boolean interrupted) {
        intakeManager.setState(IntakeManagerState.STOW);
    }


}
