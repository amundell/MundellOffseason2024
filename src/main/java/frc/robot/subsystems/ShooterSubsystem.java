// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_shooterMotorLead = new TalonFX(Constants.shooterMotorLeadID);
  private final TalonFX m_shooterMotorFollow = new TalonFX(Constants.shooterMotorFollowID);

  private final TalonFXConfiguration m_shooterMotorLeadConfiguration = new TalonFXConfiguration();
  private VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {

    var slot0Configs = m_shooterMotorLeadConfiguration.Slot0;
    slot0Configs.kP = Constants.shooterkp;
    slot0Configs.kI = Constants.shooterki;
    slot0Configs.kD = Constants.shooterkd;
    slot0Configs.kS = Constants.shooterks;
    slot0Configs.kV = Constants.shooterkv;
    slot0Configs.kA = Constants.shooterka;
    m_shooterMotorLeadConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    m_shooterMotorLead.getConfigurator().apply(m_shooterMotorLeadConfiguration);
    m_shooterMotorLead.setInverted(false);
    m_shooterMotorFollow.setInverted(true);
    m_shooterMotorFollow.setControl(new StrictFollower(Constants.shooterMotorLeadID));

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */


  public void setShooterVelocity(int rpm){
    m_velocityRequest.withVelocity(rpm);
  }

  public Command setShooterCommand(int rpm) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.run(()->setShooterVelocity(rpm),this);
  }

  public Command setShooterVoltageCommand(double power){
    return Commands.runEnd(() -> m_shooterMotorLead.setVoltage(power),() -> m_shooterMotorLead.setVoltage(0),this);
  }

  public Command stopShooterCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.run(()->setShooterVelocity(0),this);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_shooterMotorLead.setControl(m_velocityRequest);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
