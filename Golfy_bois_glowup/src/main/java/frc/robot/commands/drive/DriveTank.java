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
    private boolean isClosedLoop;

    public DriveTank(DoubleSupplier leftAxis, DoubleSupplier rightAxis, boolean isClosedLoop, Drive drive){
        this.drive = drive;
        this.leftAxis = leftAxis;
        this.rightAxis = rightAxis;
        this.isClosedLoop = isClosedLoop;

        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(isClosedLoop)
            tankDrive(leftAxis.getAsDouble(), rightAxis.getAsDouble());
        else{
            tankDriveOpen(leftAxis.getAsDouble(), rightAxis.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        tankDrive(0, 0);
    }

    public void tankDriveOpen(double left, double right){
        double leftSpeed = leftAxis.getAsDouble();
        double rightSpeed = rightAxis.getAsDouble();

        if(Math.abs(leftSpeed) < 0.1)
            leftSpeed = 0;
            
        if(Math.abs(rightSpeed) < 0.1)
            rightSpeed = 0;

        drive.setPercent(leftSpeed, rightSpeed);
    }

    public void tankDrive(double leftInput, double rightInput){
        //DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(modifyInputs(left, false), modifyInputs(right, false));

        drive.setSpeeds(new DifferentialDriveWheelSpeeds(
            -Mutil.modifyInputs(leftInput, drive.getDriveFactor()),
            -Mutil.modifyInputs(rightInput, drive.getDriveFactor())
            ));
    }

}
