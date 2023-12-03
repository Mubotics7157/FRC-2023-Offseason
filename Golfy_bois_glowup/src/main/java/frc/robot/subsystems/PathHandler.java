package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.jsontype.impl.SubTypeValidator;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PathHandler extends SubsystemBase{
    
    private static PathHandler instance = new PathHandler();

    private double bGain = 2; 
    private double zetaGain = 0.7;

    public PathHandler(){
        PathPlannerServer.startServer(5811);

        SmartDashboard.putNumber("B Gain", bGain);
        SmartDashboard.putNumber("Zeta Gain", zetaGain);
    }

    public static PathHandler getInstance(){
        return instance;
    }

    //logs to advantage scope
    public void logPath(PathPlannerTrajectory traj){
        Logger.getInstance().recordOutput("Trajectory", traj);
    }
    
    //configures the controller gains through smartdashboard
    public void configGains(){
        configGains(SmartDashboard.getNumber("B Gain", bGain), SmartDashboard.getNumber("Zeta Gain", zetaGain));
    }

    //configures the controller gains through the given parameters
    public void configGains(double bGain, double zetaGain){
        this.bGain = bGain;
        this.zetaGain = zetaGain;
    }

    //returns the command to follow the path 
    public Command followPath(PathPlannerTrajectory traj, boolean isFirstPath) {

        PathPlannerTrajectory newTraj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

        return new SequentialCommandGroup(new InstantCommand(() -> {
            logPath(newTraj);

            if(isFirstPath)
                Tracker.getInstance().setPose(newTraj.getInitialPose());
        }),
        
            new PPRamseteCommand(
                newTraj,
                Tracker.getInstance()::getPose,
                new RamseteController(bGain, zetaGain), //b , zeta
                DriveConstants.DRIVE_KINEMATICS,
                Drive.getInstance()::setSpeeds,
                false,
                Drive.getInstance()
            )
            );
    }
}
