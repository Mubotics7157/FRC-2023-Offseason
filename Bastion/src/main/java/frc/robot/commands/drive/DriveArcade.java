package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveArcade extends CommandBase{
    
    private Drive drive;
    private DoubleSupplier fwd;
    private DoubleSupplier turn;

    public DriveArcade(DoubleSupplier fwd, DoubleSupplier turn, Drive drive){
        this.drive = drive;
        this.fwd = fwd;
        this.turn = turn;

        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drive.arcadeDrive(fwd.getAsDouble(), turn.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }


}
