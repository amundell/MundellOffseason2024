// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.intakeMotorPort,MotorType.kBrushless);
  private final DigitalInput m_breakBeam1 = new DigitalInput(Constants.breakBeam1Port);

  public IntakeSubsystem() {
    m_intakeMotor.setSmartCurrentLimit(Constants.intakeMotorCurrentLimit);
    m_intakeMotor.setInverted(true);

  }

  public void Intake(){
    if (m_breakBeam1.get() == true){
      m_intakeMotor.set(Constants.intakeSpeed);
      }
    else{
      m_intakeMotor.set(0);
    }
  }
  
  public void shootIntake(){
    m_intakeMotor.set(1);
  }
  

  public void stopIntake(){
    m_intakeMotor.set(0);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */

  public boolean hasNote(){
    return !m_breakBeam1.get();
  }

  public Command runIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runEnd (
        () -> Intake(),
        () -> stopIntake());
  }

  public Command purgeIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return startEnd(
        () -> m_intakeMotor.set(-Constants.intakeSpeed),
        () -> m_intakeMotor.set(0));
  }
  
  public Command shootIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return startEnd(
        () -> shootIntake(),
        () -> m_intakeMotor.set(0));
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
