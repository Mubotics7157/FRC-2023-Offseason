package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.util.Mutil;

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
        tankDrive(leftAxis.getAsDouble(), rightAxis.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        tankDrive(0, 0);
    }

    public void tankDrive(double leftInput, double rightInput){
        //DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(modifyInputs(left, false), modifyInputs(right, false));

        drive.setSpeeds(new DifferentialDriveWheelSpeeds(
            Mutil.modifyInputs(leftInput, false),
            Mutil.modifyInputs(rightInput, false)
            ));
    }

}
