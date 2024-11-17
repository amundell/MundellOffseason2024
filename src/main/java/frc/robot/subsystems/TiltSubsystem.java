// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.MutableMeasure.mutable;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class TiltSubsystem extends SubsystemBase {
  /** Creates a new TiltSubsystem. */
  private final CANSparkMax m_tiltMotor = new CANSparkMax(Constants.tiltMotorPort, MotorType.kBrushless);
  
  private final SparkAbsoluteEncoder m_AbsoluteEncoder = m_tiltMotor.getAbsoluteEncoder();
  private final RelativeEncoder m_RelativeEncoder = m_tiltMotor.getEncoder();
  private double holdLocation = 45;
  
  private final SparkPIDController m_PidController = m_tiltMotor.getPIDController();

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  


  public TiltSubsystem() {
    m_tiltMotor.setSmartCurrentLimit(Constants.TiltCurrentLimit);

    //m_PidController.setFeedbackDevice(m_AbsoluteEncoder);

    m_tiltMotor.setInverted(false);
    m_AbsoluteEncoder.setPositionConversionFactor(360);
    m_AbsoluteEncoder.setInverted(false);
    m_AbsoluteEncoder.setZeroOffset(Constants.TiltOffset);
    m_RelativeEncoder.setPositionConversionFactor(1/Constants.tiltGearRatio*360); //Position in degrees
    m_RelativeEncoder.setVelocityConversionFactor(1/Constants.tiltGearRatio/60);  //Position in rot/s

    m_tiltMotor.enableSoftLimit(SoftLimitDirection.kForward,true);
    m_tiltMotor.enableSoftLimit(SoftLimitDirection.kReverse,true);
    m_tiltMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.TiltTopLimt);
    m_tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.TiltBotLimit);

    m_PidController.setFeedbackDevice(m_AbsoluteEncoder);

    m_PidController.setP(Constants.tiltkP);
    m_PidController.setI(Constants.tiltkI);
    m_PidController.setD(Constants.tiltkD);
    m_PidController.setIZone(Constants.tiltkIz);
    m_PidController.setFF(Constants.tiltkFF);
    m_PidController.setOutputRange(Constants.tiltkMinOutput, Constants.tiltkMaxOutput);

    m_RelativeEncoder.setPosition(m_AbsoluteEncoder.getPosition());
  }

    private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.of(.1).per(Seconds.of(1)), Volts.of(2.5), Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {m_tiltMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the tilt motor.
                log.motor("tilt_motor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_tiltMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(m_RelativeEncoder.getPosition()/360, Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(m_RelativeEncoder.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Command tiltwithJoystickCommand(DoubleSupplier joystick){
    return run(
      () -> {
        m_tiltMotor.set(joystick.getAsDouble()*-0.25);
        holdLocation = m_AbsoluteEncoder.getPosition();
      });
  }

  public Command tiltPIDControl(double degrees){
    return run(
      () -> {
        m_PidController.setReference(degrees, ControlType.kPosition);
        holdLocation = degrees;
      }
    );
  }

  public Command autoTilt(DoubleSupplier degrees){
    return run(
      () -> {
        m_PidController.setReference(degrees.getAsDouble(), ControlType.kPosition);
        holdLocation = degrees.getAsDouble();
      }
    );
  }


    public Command tiltPIDHold(){
    return run(
      () -> {
        m_PidController.setReference(holdLocation, ControlType.kPosition);
      }
    );
  }

  public Command tiltwithVoltageCommand(double voltage){
    return run(
      () -> {
        m_tiltMotor.setVoltage(voltage);
      }
    );
  }
  

  public Command syncTiltEncodersCommand(){
    return runOnce(
      () -> {
        m_RelativeEncoder.setPosition(m_AbsoluteEncoder.getPosition());
      });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
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
    //log to glass for debugging
    SmartDashboard.putNumber("tilt motor position", m_RelativeEncoder.getPosition());
    SmartDashboard.putNumber("tilt abs enc position", m_AbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("hold location", holdLocation);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }



}
