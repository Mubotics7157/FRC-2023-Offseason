package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Tracker extends SubsystemBase{
    
    private static Tracker instance = new Tracker();

    private final Drive drive = Drive.getInstance();

    private final Field2d m_field = new Field2d();
    
    private DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(
        DriveConstants.DRIVE_KINEMATICS,
        drive.getHeading(),
        drive.getLeftDistance(),
        drive.getRightDistance(),
        new Pose2d(),

    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            .1,
            .1,
            .1
        ),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill( //vision boi
            .5,
            .5,
            2.5
        )
    );

    public Tracker(){

    }

    @Override
    public void periodic() {
        updatePose();
        logData();
        //m_field.setRobotPose(getPose());
    }


    public static Tracker getInstance(){
        return instance;
    }

    public void logData(){
        Logger.getInstance().recordOutput("Estimator Pose", getPose());
    }

    private void updatePose(){
        estimator.updateWithTime(Timer.getFPGATimestamp(), drive.getHeading(), drive.getLeftDistance(), drive.getRightDistance());
    }

    public void setPose(Pose2d pose){
        estimator.resetPosition(drive.getHeading(), drive.getLeftDistance(), drive.getRightDistance(), pose);
    }

    public void resetPose(){
        setPose(new Pose2d());
    }

    public Pose2d getPose(){
        return estimator.getEstimatedPosition();
    }
    public void resetHeading(){
        estimator.resetPosition(drive.getHeading(), drive.getLeftDistance(), drive.getRightDistance(), new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    }
}
