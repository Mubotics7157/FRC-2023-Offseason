package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PathHandler;

public class TestAuto extends SequentialCommandGroup{

    public TestAuto(PathHandler handler){

        PathPlannerTrajectory partOne = PathPlanner.loadPath("Test", PathPlanner.getConstraintsFromPath("Test"));

        addCommands(
            handler.followTrajectoryCommand(partOne, true)
        );
    }
}
