package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.util.Mutil;

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
        arcadeDrive(fwd.getAsDouble(), turn.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        arcadeDrive(0, 0);
    }

    public void arcadeDrive(double fwd, double turn){
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(
            Mutil.modifyInputs(fwd, false),
            0.0,
            Mutil.modifyInputs(turn, true)));

        drive.setSpeeds(wheelSpeeds);
    }


}
