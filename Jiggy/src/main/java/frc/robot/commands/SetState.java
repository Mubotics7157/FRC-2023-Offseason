package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetState<T> extends CommandBase{
    
    private Consumer<T> method;
    private T stateType;

    public SetState(Consumer<T> method, T stateType){
        this.method = method;
        this.stateType = stateType;
    }

    @Override
    public void initialize() {
        method.accept(stateType);
    } 
}
