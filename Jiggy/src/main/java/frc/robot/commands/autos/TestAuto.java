package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PathHandler;

public class TestAuto extends SequentialCommandGroup{

    public TestAuto(PathHandler handler){

        PathPlannerTrajectory partOne = PathPlanner.loadPath("TestPartOne", PathPlanner.getConstraintsFromPath("TestPartOne"));
        PathPlannerTrajectory partTwo = PathPlanner.loadPath("TestPartTwo", PathPlanner.getConstraintsFromPath("TestPartTwo"));
        PathPlannerTrajectory partThree = PathPlanner.loadPath("TestPartThree", PathPlanner.getConstraintsFromPath("TestPartThree"));
        PathPlannerTrajectory partFour = PathPlanner.loadPath("TestPartFour", PathPlanner.getConstraintsFromPath("TestPartFour"));

        addCommands(
            new InstantCommand(handler::configGains),
            handler.followPath(partOne, true),
            handler.followPath(partTwo, false),
            handler.followPath(partThree, false),
            handler.followPath(partFour, false)
        );
    }
}
