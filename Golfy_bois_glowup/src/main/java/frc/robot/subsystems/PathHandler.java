package frc.robot.subsystems;

import com.fasterxml.jackson.databind.jsontype.impl.SubTypeValidator;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PathHandler extends SubsystemBase{
    
    public PathHandler(){
        PathPlannerServer.startServer(5811);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

        PathPlannerTrajectory newTraj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

        return new SequentialCommandGroup(new InstantCommand(() -> {
            if(isFirstPath)
                Tracker.getInstance().setPose(newTraj.getInitialPose());
        }),
        
            new PPRamseteCommand(
                newTraj,
                Tracker.getInstance()::getPose,
                new RamseteController(2, 0.7), //b , zeta
                DriveConstants.DRIVE_KINEMATICS,
                Drive.getInstance()::setSpeeds,
                false,
                Drive.getInstance()
            )
            );
    }
}
