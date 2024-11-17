// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final TiltSubsystem m_tiltSubsystem = new TiltSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  //private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort); // My joystick
  private final CommandXboxController joystick3 = new CommandXboxController(OperatorConstants.kThirdJoystickPort); // My joystick


  /** The container for the robot. Cotains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings.
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                                 OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                                 OperatorConstants.RIGHT_X_DEADBAND),
                                                                   m_driverController.getHID()::getYButtonPressed,
                                                                   m_driverController.getHID()::getAButtonPressed,
                                                                   m_driverController.getHID()::getXButtonPressed,
                                                                   m_driverController.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot

    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX() * -1.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity: driveFieldOrientedDirectAngleSim);

    Command AutoTilt = m_tiltSubsystem.autoTilt(()->Constants.distanceToAngle.get(drivebase.getDistanceToSpeaker()));
   
    m_tiltSubsystem.setDefaultCommand(AutoTilt);

    NamedCommands.registerCommand("Intake", Commands.parallel(m_intakeSubsystem.runIntakeCommand(),m_tiltSubsystem.tiltPIDControl(Constants.IntakeAngle)).until(()->m_intakeSubsystem.hasNote()));
    NamedCommands.registerCommand("Shoot", m_intakeSubsystem.shootIntakeCommand().withTimeout(1));
    NamedCommands.registerCommand("PodiumAim", Commands.parallel(m_tiltSubsystem.tiltPIDControl(Constants.PodiumShotAngle),m_shooterSubsystem.setShooterVoltageCommand(10)).withTimeout(2));
    NamedCommands.registerCommand("CloseAim", Commands.parallel(m_tiltSubsystem.tiltPIDControl(Constants.WallShotAngle),m_shooterSubsystem.setShooterVoltageCommand(10)).withTimeout(1.5));
    NamedCommands.registerCommand("AutoAim", Commands.parallel(AutoTilt, drivebase.aimAtSpeaker(()->0,()->0,1),m_shooterSubsystem.setShooterVoltageCommand(10)).withTimeout(2));

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
 
    m_driverController.back().onTrue(drivebase.zeroGyroCommand());
    m_driverController.back().onTrue(m_tiltSubsystem.syncTiltEncodersCommand());

    m_driverController.rightBumper().whileTrue(m_intakeSubsystem.runIntakeCommand());
    m_driverController.rightBumper().whileTrue(m_tiltSubsystem.tiltPIDControl(Constants.IntakeAngle));

    m_driverController.leftBumper().whileTrue(m_intakeSubsystem.purgeIntakeCommand().beforeStarting(Commands.waitSeconds(1.5)));
    m_driverController.leftBumper().onTrue(m_tiltSubsystem.tiltPIDControl(Constants.PurgeAngle));

    m_driverController.rightTrigger().whileTrue(m_intakeSubsystem.shootIntakeCommand());

    m_driverController.a().whileTrue(m_tiltSubsystem.tiltPIDControl(Constants.WallShotAngle));
    m_driverController.a().whileTrue(m_shooterSubsystem.setShooterVoltageCommand(10));

    m_driverController.b().whileTrue(m_tiltSubsystem.tiltPIDControl(Constants.PodiumShotAngle));
    m_driverController.b().whileTrue(m_shooterSubsystem.setShooterVoltageCommand(10));

    m_driverController.y().whileTrue(m_shooterSubsystem.setShooterVoltageCommand(10));

    m_driverController.x().whileTrue(drivebase.aimAtSpeaker(()->m_driverController.getLeftX(),()->m_driverController.getLeftY(),1));
    m_driverController.x().whileTrue(m_shooterSubsystem.setShooterVoltageCommand(10));



    new Trigger(() -> DriverRightYAxisUsed()).whileTrue(m_tiltSubsystem.tiltwithJoystickCommand(()-> m_driverController.getRightY()));

        /* Bindings for characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */

    // Temporary buttons
    joystick3.start().and(joystick3.y()).whileTrue(m_tiltSubsystem.sysIdQuasistatic(Direction.kForward));
    joystick3.start().and(joystick3.x()).whileTrue(m_tiltSubsystem.sysIdQuasistatic(Direction.kReverse));
    joystick3.back().and(joystick3.y()).whileTrue(m_tiltSubsystem.sysIdDynamic(Direction.kForward));
    joystick3.back().and(joystick3.x()).whileTrue(m_tiltSubsystem.sysIdDynamic(Direction.kReverse));
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Front3");
  }

  public Boolean DriverRightYAxisUsed() {
    SmartDashboard.putBoolean("Right Axis Used", Math.abs(m_driverController.getRightY())> 0.05);
    return (Math.abs(m_driverController.getRightY())> 0.05);
  }

}
