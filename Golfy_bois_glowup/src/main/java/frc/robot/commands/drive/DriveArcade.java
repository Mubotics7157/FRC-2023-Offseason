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
    private boolean isClosedLoop;

    public DriveArcade(DoubleSupplier fwd, DoubleSupplier turn, boolean isClosedLoop, Drive drive){
        this.drive = drive;
        this.fwd = fwd;
        this.turn = turn;
        this.isClosedLoop = isClosedLoop;

        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(isClosedLoop)
            arcadeDrive(fwd.getAsDouble(), turn.getAsDouble());
        else
            arcadeDriveOpen(fwd.getAsDouble(), turn.getAsDouble(), 1);
    }

    @Override
    public void end(boolean interrupted) {
        arcadeDriveOpen(0, 0, 1);
    }

    public void arcadeDrive(double fwd, double turn){
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(
            Mutil.modifyInputs(fwd, false),
            0.0,
            Mutil.modifyInputs(turn, true)));

        drive.setSpeeds(wheelSpeeds);
    }

    public void arcadeDriveOpen(double fwd, double turn, double turnFactor){
        double rightSpeed = fwd + (turn * turnFactor);
        double leftSpeed = fwd - (turn * turnFactor);

        drive.setPercent(leftSpeed, rightSpeed);
    }


}
