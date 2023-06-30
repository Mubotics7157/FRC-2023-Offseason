package frc.robot.subsystems;

import javax.lang.model.element.ModuleElement.DirectiveKind;

import org.opencv.core.RotatedRect;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.CommonConversions;

public class Drive extends SubsystemBase{
    
    private static Drive instance = new Drive();

    private WPI_TalonFX leftMaster = new WPI_TalonFX(DriveConstants.DEVICE_ID_LEFT_MASTER);
    private WPI_TalonFX leftSlave = new WPI_TalonFX(DriveConstants.DEVICE_ID_LEFT_SLAVE);

    private WPI_TalonFX rightMaster = new WPI_TalonFX(DriveConstants.DEVICE_ID_RIGHT_MASTER);
    private WPI_TalonFX rightSlave = new WPI_TalonFX(DriveConstants.DEVICE_ID_RIGHT_SLAVE);

    private WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.DEVICE_ID_PIGEON);

    public Drive(){
        gyro.reset();

        leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(false);
        leftSlave.setInverted(InvertType.FollowMaster);
        rightMaster.setInverted(true);
        rightSlave.setInverted(InvertType.FollowMaster);

        leftMaster.setNeutralMode(DriveConstants.NEUTRAL_MODE);
        leftSlave.setNeutralMode(DriveConstants.NEUTRAL_MODE);
        rightMaster.setNeutralMode(DriveConstants.NEUTRAL_MODE);
        rightSlave.setNeutralMode(DriveConstants.NEUTRAL_MODE);

        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);

        leftMaster.configPeakOutputForward(1);
        leftMaster.configPeakOutputReverse(-1);
        rightMaster.configPeakOutputForward(1);
        rightMaster.configPeakOutputReverse(-1);

    }

    public static Drive getInstance(){
        if(instance == null)
            return new Drive();
        else
            return instance;
    }

    @Override
    public void periodic() {
        
    }

    public void logData(){

    }

    public Rotation2d getHeading(){
        return gyro.getRotation2d();
    }

    public void resetHeading(){
        gyro.reset();
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
        double leftSpeed = CommonConversions.metersPerSecToStepsPerDecisec(speeds.leftMetersPerSecond, DriveConstants.WHEEL_DIAMETER_METERS);
        double rightSpeed = CommonConversions.metersPerSecToStepsPerDecisec(speeds.rightMetersPerSecond, DriveConstants.WHEEL_DIAMETER_METERS);

        leftMaster.set(ControlMode.Velocity, leftSpeed);
        rightMaster.set(ControlMode.Velocity, rightSpeed);
    }

    public void arcadeDrive(double fwd, double turn){
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(
            modifyInputs(fwd, false),
            0.0,
            modifyInputs(turn, true)));

        setSpeeds(wheelSpeeds);
    }

    public void tankDrive(double left, double right){
        //DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(modifyInputs(left, false), modifyInputs(right, false));

        setSpeeds(new DifferentialDriveWheelSpeeds(
            modifyInputs(left, false),
            modifyInputs(right, false)
            ));
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
            if(isFirstPath)
                Tracker.getInstance().setPose(traj.getInitialPose());

        return new SequentialCommandGroup(
            new PPRamseteCommand(
                traj, 
                Tracker.getInstance()::getPose, // Pose supplier
                new RamseteController(),
                DriveConstants.DRIVE_FEEDFORWARD,
                DriveConstants.DRIVE_KINEMATICS, // DifferentialDriveKinematics
                this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                this::outputVolts, // Voltage biconsumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            ));
    }
    
    public void outputVolts(double leftVolts, double rightVolts){
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
    }

    public double getLeftDistance(){
        return CommonConversions.stepsToMeters(leftMaster.getSelectedSensorPosition());
    }

    public double getRightDistance(){
        return CommonConversions.stepsToMeters(leftSlave.getSelectedSensorPosition());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(
            CommonConversions.stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity()),
            CommonConversions.stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity())
        );
    }

    private double modifyInputs(double val, boolean isRot){
        if(isRot){
            if(Math.abs(val) < 0.1){
                val = 0;
            }
            //return val*DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
            //val = Math.copySign(Math.pow(val, 2),val);
            return val * DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
        }
        else{
            if(Math.abs(val) < 0.1){
                val = 0;
            }
            //return val*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
            //val = Math.copySign(Math.pow(val, 2),val);
            return val * DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        }
    }
}
