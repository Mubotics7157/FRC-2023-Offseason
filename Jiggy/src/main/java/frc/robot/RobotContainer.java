// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetState;
import frc.robot.commands.autos.TestAuto;
import frc.robot.commands.drive.ChangeFactors;
import frc.robot.commands.drive.DriveArcade;
import frc.robot.commands.drive.DriveTank;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PathHandler;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.Intaking.Intake;
import frc.robot.subsystems.Intaking.Intake.IntakeState;
import frc.robot.subsystems.Shooting.Hood;
import frc.robot.subsystems.Shooting.Shooter;
import frc.robot.subsystems.Shooting.ShooterManager;
import frc.robot.subsystems.Shooting.Hood.HoodState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drive drive = Drive.getInstance();
  private final Tracker tracker = Tracker.getInstance();
  private final PathHandler handler = PathHandler.getInstance();
  private final Intake intake = Intake.getInstance();
  //private final ShooterManager shooterManager = ShooterManager.getInstance();
  //private final Hood hood = Hood.getInstance();
  //private final Shooter shooter = Shooter.getInstance();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DEVICE_ID_DRIVER_CONTROLLER);

  private final CommandJoystick m_operatorController = 
      new CommandJoystick(OperatorConstants.DEVICE_ID_OPERATOR_CONTROLLER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drive.setDefaultCommand(new DriveArcade(m_driverController::getLeftY, m_driverController::getRightX, true, drive));
    //drive.setDefaultCommand(new DriveTank(m_driverController::getLeftY, m_driverController::getRightY, true, drive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.leftTrigger().onTrue(new SequentialCommandGroup(new SetState<IntakeState>(intake::setState, IntakeState.CUSTOM), new InstantCommand(intake::configGains)));
    m_driverController.leftTrigger().onFalse(new SetState<IntakeState>(intake::setState, IntakeState.OFF));

    m_driverController.a().onTrue(new InstantCommand(intake::zeroEncoder));
    //m_driverController.a().onTrue(new InstantCommand(tracker::resetPose));
    //m_driverController.x().onTrue(new InstantCommand(drive::configGains));



    //m_driverController.leftTrigger().whileTrue(new ChangeFactors(2, Math.PI, drive));
    //m_driverController.b().onTrue(new InstantCommand(drive::resetEncoders));

    //m_driverController.a().onTrue(new SetState<HoodState>(hood::setState, HoodState.DYNAMIC));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new TestAuto(handler);
  }
}
