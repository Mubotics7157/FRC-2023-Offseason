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
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants{

    public static final double driveKS = 0.12256;
    public static final double driveKV = 2.3396;
    public static final double driveKA = 0.19701;
    public static final double driveKP = 0.0018763;

    public static final int DEVICE_ID_LEFT_MASTER = 1;
    public static final int DEVICE_ID_LEFT_SLAVE = 2;
    public static final int DEVICE_ID_RIGHT_MASTER = 3;
    public static final int DEVICE_ID_RIGHT_SLAVE = 4;

    public static final int DEVICE_ID_PIGEON = 5;

    public static final double DRIVE_GEAR_RATIO = 10.75;

    public static final String DRIVE_CANIVORE_ID = "drive";

    public static final double MAX_TANGENTIAL_VELOCITY = 4.3;
    public static final double MAX_TELE_TANGENTIAL_VELOCITY = 4.3;
    public static final double MAX_TELE_ANGULAR_VELOCITY = Math.PI;

    public static final double WHEELBASE_LENGTH = Units.inchesToMeters(20.75);
    public static final double WHEELBASE_WIDTH = Units.inchesToMeters(20.75);

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(WHEELBASE_WIDTH);

    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
  }

  public static class HoodConstants{

    public static final int DEVICE_ID_HOOD = 5;

    public static final double HOOD_KP = 0.075;

    public static final int HOOD_GEARING = 5;

    public static final int DEVICE_ID_LIMIT_SWITCH = 2;
  }

  public static class TurretConstants{

    public static final int DEVICE_ID_TURRET = 6;

    public static final double TRACKING_KP = 0;
    public static final double TRACKING_KI = 0;
    public static final double TRACKING_KD = 0;

    public static final double TURRET_KP = 0.075;

    public static final int TURRET_GEARING = 15;

    public static final Rotation2d TURRET_MAX = Rotation2d.fromDegrees(270);

    public static final int DEVICE_ID_LIMIT_SWITCH = 1;
  }

  public static class ShooterConstants{
    
    public static final int DEVICE_ID_SHOOTER = 7;

    public static final double SHOOTER_GEARING = 5;

    public static final double SHOOTER_KP = 0.04;

    public static final double SHOOTER_KF = 0.00012;
    
  }

  public static class VisionConstants{
    public static final String TURRET_LL_NAME = "turret";

    public static final double TURRET_LL_HEIGHT_METERS = 1;
    
    public static final double TARGET_HEIGHT_METERS = 4;

    public static final double TURRET_LL_MOUNTING_PITCH_RADIANS = Units.degreesToRadians(20);
  }

}
