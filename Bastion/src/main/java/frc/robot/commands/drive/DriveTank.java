package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveTank extends CommandBase{
    
    private Drive drive;
    private DoubleSupplier leftAxis;
    private DoubleSupplier rightAxis;

    public DriveTank(DoubleSupplier leftAxis, DoubleSupplier rightAxis, Drive drive){
        this.drive = drive;
        this.leftAxis = leftAxis;
        this.rightAxis = rightAxis;

        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drive.tankDrive(leftAxis.getAsDouble(), rightAxis.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drive.tankDrive(0, 0);
    }


}
