package frc.robot.subsystems.Intaking;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intaking.Intake.IntakeState;
import frc.robot.subsystems.Intaking.Spindexer.SpindexerState;

public class IntakeManager extends SubsystemBase{

    public enum IntakeManagerState{
        OFF,
        INTAKING,
        OUTTAKING,
        SHOOTING,
        STOW
    }
    
    private static IntakeManager instance = new IntakeManager();

    private final Intake intake = Intake.getInstance();
    private final Spindexer spindexer = Spindexer.getInstance();

    private IntakeManagerState currentState = IntakeManagerState.STOW;

    public static IntakeManager getInstance(){
        return instance;
    }

    public void setState(IntakeManagerState newState){

        if(currentState != newState)
            currentState = newState;
        
        switch(currentState){
            case OFF:
                intake.setState(IntakeState.STOW);
                spindexer.setState(SpindexerState.OFF);
                break;

            case INTAKING:
                intake.setState(IntakeState.INTAKING);
                spindexer.setState(SpindexerState.INTAKING);
                break;
            
            case OUTTAKING:
                intake.setState(IntakeState.OUTTAKING);
                break;
            
            case SHOOTING:
                intake.setState(IntakeState.STOW);
                spindexer.setState(SpindexerState.SHOOTING);
                break;

            case STOW:
                intake.setState(IntakeState.STOW);
                spindexer.setState(SpindexerState.IDLE);
                break;
        }
    }

    public IntakeManagerState getState(){
        return currentState;
    }

    
    
}
