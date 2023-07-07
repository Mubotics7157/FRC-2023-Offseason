package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class Zero extends CommandBase{

    SuperStructure superStructure;

    SuperStructureState previousState;
    
    public Zero(SuperStructure superStructure){
        this.superStructure = superStructure;

        addRequirements(superStructure);
    }
    
    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.ZERO);
        previousState = superStructure.getState();
    }

    @Override
    public boolean isFinished() {
        return superStructure.isZeroed();
    }

    @Override
    public void end(boolean interrupted) {
        superStructure.setState(previousState);
    }
}
