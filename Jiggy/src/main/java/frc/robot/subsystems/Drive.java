package frc.robot.subsystems;

import javax.lang.model.element.ModuleElement.DirectiveKind;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.RotatedRect;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.CommonConversions;
import frc.robot.util.LiveNumber;
import frc.robot.util.Mutil;

public class Drive extends SubsystemBase{
    
    private static Drive instance = new Drive();

    private WPI_TalonFX leftMaster = new WPI_TalonFX(DriveConstants.DEVICE_ID_LEFT_MASTER);
    private WPI_TalonFX leftSlave = new WPI_TalonFX(DriveConstants.DEVICE_ID_LEFT_SLAVE);

    private WPI_TalonFX rightMaster = new WPI_TalonFX(DriveConstants.DEVICE_ID_RIGHT_MASTER);
    private WPI_TalonFX rightSlave = new WPI_TalonFX(DriveConstants.DEVICE_ID_RIGHT_SLAVE);

    private WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.DEVICE_ID_PIGEON);

    private double driveFactor = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
    private double turnFactor = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;

    private LiveNumber driveP = new LiveNumber("drive kP", DriveConstants.driveKP);

    private SlewRateLimiter leftLimiter = new SlewRateLimiter(4, -4, 0);
    private SlewRateLimiter rightLimiter = new SlewRateLimiter(4, -4, 0);
    public Drive(){
        gyro.reset();

        configMotors();
    }

    public static Drive getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        logData();
    }

    public void logData(){
        Logger.getInstance().recordOutput("Drive/left speed", getLeftSpeed());
        Logger.getInstance().recordOutput("Drive/right speed", getRightSpeed());

        //SmartDashboard.putNumber("left speed", getLeftSpeed());
        //SmartDashboard.putNumber("right speed", getRightSpeed());

        //SmartDashboard.putNumber("left distance", getLeftDistance());
        //SmartDashboard.putNumber("right distance", getRightDistance());

        //SmartDashboard.putNumber("gyro heading", getHeading().getDegrees());
    }

    public Rotation2d getHeading(){
        return gyro.getRotation2d();
    }

    public void resetHeading(){
        gyro.reset();
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
        setSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
    }

    public void setPercent(double left, double right){
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }
    
    public void setSpeeds(double leftMPS, double rightMPS){
        double modifiedLeft = leftLimiter.calculate(leftMPS);
        double modifiedRight = rightLimiter.calculate(rightMPS);

        double leftSpeed = CommonConversions.metersPerSecToStepsPerDecisec(modifiedLeft, DriveConstants.WHEEL_DIAMETER_METERS);
        double rightSpeed = CommonConversions.metersPerSecToStepsPerDecisec(modifiedRight, DriveConstants.WHEEL_DIAMETER_METERS);

        //SmartDashboard.putNumber("wanted left", leftMPS);
        //SmartDashboard.putNumber("wanted right", rightMPS);
        Logger.getInstance().recordOutput("Drive/left speed wanted", modifiedLeft);
        Logger.getInstance().recordOutput("Drive/right speed wanted", modifiedRight);

        //SmartDashboard.putNumber("left error", Math.abs(leftMPS - getLeftSpeed()));
        //SmartDashboard.putNumber("right error", Math.abs(rightMPS - getRightSpeed()));
        Logger.getInstance().recordOutput("Drive/left error", modifiedLeft - getLeftSpeed());
        Logger.getInstance().recordOutput("Drive/right errr", modifiedRight - getRightSpeed());

        if(leftSpeed == 0)
            leftMaster.set(ControlMode.PercentOutput, 0);
        else
            leftMaster.set(ControlMode.Velocity, leftSpeed, 
                DemandType.ArbitraryFeedForward, DriveConstants.DRIVE_FEEDFORWARD.calculate(modifiedLeft) / 12);
        
        if(rightSpeed == 0)
            rightMaster.set(ControlMode.PercentOutput, 0);
        else
            rightMaster.set(ControlMode.Velocity, rightSpeed,
                DemandType.ArbitraryFeedForward, DriveConstants.DRIVE_FEEDFORWARD.calculate(modifiedRight) / 12);
    }

    public double getLeftDistance(){
        return CommonConversions.stepsToMeters(leftMaster.getSelectedSensorPosition());
    }

    public double getRightDistance(){
        return CommonConversions.stepsToMeters(rightMaster.getSelectedSensorPosition());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(
            getLeftSpeed(),
            getRightSpeed()
        );
    }

    private double getLeftSpeed(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity());
    }

    private double getRightSpeed(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity());
    }

    public void setDriveFactor(double newFactor){
        driveFactor = newFactor;
    }

    public double getDriveFactor(){
        return driveFactor;
    }

    public void setTurnFactor(double newFactor){
        turnFactor = newFactor;
    }

    public double getTurnFactor(){
        return turnFactor;
    }

    public void resetEncoders(){
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    public void setCoast(){
        setNeutralMode(NeutralMode.Coast);
    }

    public void setBrake(){
        setNeutralMode(NeutralMode.Brake);
    }

    public void setNeutralMode(NeutralMode mode){
        leftMaster.setNeutralMode(mode);
        leftSlave.setNeutralMode(mode);
        rightMaster.setNeutralMode(mode);
        rightSlave.setNeutralMode(mode);
    }

    public void configGains(){
        leftMaster.config_kP(0, driveP.get());
        //leftMaster.config_kD(0, SmartDashboard.getNumber("drive kD", 0));
        //leftMaster.config_kF(0, SmartDashboard.getNumber("drive kF", 0));

        rightMaster.config_kP(0, driveP.get());
        //rightMaster.config_kD(0, SmartDashboard.getNumber("drive kD", 0));
        //rightMaster.config_kF(0, SmartDashboard.getNumber("drive kF", 0));

    }

    private void configMotors(){

        leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(true);
        leftSlave.setInverted(InvertType.FollowMaster);
        rightMaster.setInverted(false);
        rightSlave.setInverted(InvertType.FollowMaster);

        leftMaster.setNeutralMode(DriveConstants.NEUTRAL_MODE);
        leftSlave.setNeutralMode(DriveConstants.NEUTRAL_MODE);
        rightMaster.setNeutralMode(DriveConstants.NEUTRAL_MODE);
        rightSlave.setNeutralMode(DriveConstants.NEUTRAL_MODE);

        leftMaster.configVoltageCompSaturation(12);
        rightMaster.configVoltageCompSaturation(12);
        leftMaster.enableVoltageCompensation(true);
        rightMaster.enableVoltageCompensation(true);

        leftMaster.config_kP(0, DriveConstants.driveKP);
        rightMaster.config_kP(0, DriveConstants.driveKP);
        
        //leftMaster.config_kD(0, 0);
        //rightMaster.config_kD(0, 0);
        
        //leftMaster.config_kF(0, DriveConstants.driveKF);
        //rightMaster.config_kF(0, DriveConstants.driveKF);

        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);

        leftMaster.configPeakOutputForward(1);
        leftMaster.configPeakOutputReverse(-1);

        rightMaster.configPeakOutputForward(1);
        rightMaster.configPeakOutputReverse(-1);
    }

}
