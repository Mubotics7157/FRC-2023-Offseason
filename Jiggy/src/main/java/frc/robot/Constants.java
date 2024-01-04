// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.RotatedRect;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int DEVICE_ID_DRIVER_CONTROLLER = 0;
    public static final int DEVICE_ID_OPERATOR_CONTROLLER = 1;
  }

  public static class DriveConstants{

    public static final double driveKS = 0.14414;//0.14721;//0.093307;//0.12711;
    public static final double driveKV = 1.7697;//1.7561;//0.87933;//1.7819;
    public static final double driveKA = 0.70828;//0.1356;//0.069662;//0.17246;
    //public static final double driveKP = 0.034004;

    public static final double driveKP = 0.18096;//0.019307;//0.0050532;//0.003;
    public static final double driveKF = 0.0468;


    public static final int DEVICE_ID_LEFT_MASTER = 13;
    public static final int DEVICE_ID_LEFT_SLAVE = 12;
    public static final int DEVICE_ID_RIGHT_MASTER = 11;
    public static final int DEVICE_ID_RIGHT_SLAVE = 10;

    public static final int DEVICE_ID_PIGEON = 14;

    public static final double DRIVE_GEAR_RATIO = 8;

    public static final String DRIVE_CANIVORE_ID = "drive";

    public static final double MAX_TANGENTIAL_VELOCITY = 6.57;
    public static final double MAX_TELE_TANGENTIAL_VELOCITY = 6.57;

    public static final double MAX_TELE_ANGULAR_VELOCITY = 2 * Math.PI;

    public static final double WHEELBASE_LENGTH = Units.inchesToMeters(29);
    public static final double WHEELBASE_WIDTH = Units.inchesToMeters(29);

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.09);

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(WHEELBASE_WIDTH);

    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
  }

  public static class HoodConstants{

    public static final int DEVICE_ID_HOOD = 21;

    public static final double HOOD_KP = 0.22;

    public static final int HOOD_GEARING = 15;
    //this isnt the actual gearing rotation its just to make the units smaller for easier reading

    public static final int DEVICE_ID_LIMIT_SWITCH = 0;
    public static final boolean MAG_DETECTED = false;
  }

  public static class TurretConstants{

    public static final int DEVICE_ID_TURRET = 16;

    public static final double TRACKING_KP = 0.03;
    public static final double TRACKING_KI = 0;
    public static final double TRACKING_KD = 0;

    public static final double TURRET_KP = 0.03;

    public static final double TURRET_GEARING = 210;

    public static final Rotation2d TURRET_MAX = Rotation2d.fromDegrees(270);

    public static final int DEVICE_ID_LIMIT_SWITCH = 1;
    public static final boolean MAG_DETECTED = false;
  }

  public static class ShooterConstants{
    
    public static final int DEVICE_ID_SHOOTER_MASTER = 19;
    public static final int DEVICE_ID_SHOOT_SLAVE = 20;

    public static final double SHOOTER_KP = 0.00001;
    public static final double SHOOTER_KF = 0.00019;
    
  }

  public static class SpindexerConstants{
    public static final int DEVICE_ID_SPINDEXER = 15;

    public static final double SPINDEXER_GEARING = 15;

    public static final double SPINDEXER_KP = 0;
    public static final double SPINDEXER_KD = 0;
    public static final double SPINDEXER_KF = 0;

    public static final double SPINDEXER_RAMP_RATE = 4;

    public static final double IDLE_SPEED = 0.1;
    public static final double INTAKING_SPEED = 3000;
    public static final double SHOOTING_SPEED = 2000;
  }

  public static class IntakeConstants{
    public static final int DEVICE_ID_INTAKE = 18;
    public static final int DEVICE_ID_ACTUATOR = 17;

    public static final double ACTUATOR_GEAR_RATIO = 8;

    public static final double ACTUATOR_STOW = -7600;
    public static final double ACTUATOR_DOWN = -1500;

    public static final double ACTUATOR_ACCELERATION = 50000;
    public static final double ACTUATOR_CRUISE_VELOCITY = 50000;

    public static final double ACTUATOR_KP = 0.05;
    public static final double ACTUATOR_KD = 0;

    public static final double INTAKE_KP = 0.005;
    public static final double INTAKE_KF = 0.076;

    public static final double INTAKE_SPEED = 5700;
    public static final double OUTTAKE_SPEED = -5700;
  }

  public static class ThroatConstants{
    public static final int DEVICE_ID_THROAT = 22;
    public static final double THROAT_SHOOTING_SPEED = 0.5;
    public static final double THROAT_UNCLOG_SPEED = -0.2;
  }

  public static class Setpoints{
    public static final double HOOD_STOW = 1;
    public static final Rotation2d TURRET_STOW = Rotation2d.fromDegrees(0);
  }

  public static class VisionConstants{
    public static final String TURRET_LL_NAME = "turret";

    public static final double TURRET_LL_HEIGHT_METERS = Units.inchesToMeters(23.359);
    
    public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(6); //change this for reality

    public static final double TURRET_LL_MOUNTING_PITCH_RADIANS = Units.degreesToRadians(30);
  }

}
