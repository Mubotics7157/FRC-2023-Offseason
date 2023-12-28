package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public class ChangeFactors extends CommandBase{
    
    private Drive drive;
    private double driveFactor, turnFactor;

    private double previousDrive, previousTurn;

    public ChangeFactors(double driveFactor, double turnFactor, Drive drive){
        this.drive = drive;
        this.driveFactor = driveFactor;
        this.turnFactor = turnFactor;
    }

    @Override
    public void initialize() {
        previousDrive = drive.getDriveFactor();
        previousTurn = drive.getTurnFactor();

        drive.setDriveFactor(driveFactor);
        drive.setTurnFactor(turnFactor);

    }

    @Override
    public void end(boolean interrupted) {
        drive.setDriveFactor(previousDrive);
        drive.setTurnFactor(previousTurn);
    }
}
