// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
      // Maximum speed of the robot in meters per second, used to limit acceleration.


  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kThirdJoystickPort = 2;
    public static final double RIGHT_Y_DEADBAND = 0.1;
  }

  //Tilt Constants
  public static int tiltMotorPort = 9;
  public static double tiltGearRatio = 5*5*5*18/15;
  public static final int TiltCurrentLimit = 30;
  public static final double TiltOffset = 190;
  public static final float TiltBotLimit = 40; //degrees
  public static final float TiltTopLimt = 105; //degrees

  public static final int WallShotRPM = 5000;
  public static final double WallShotAngle = 105;

  public static final int PodiusmShotRPM = 5000;
  public static final double PodiumShotAngle = 55;

  public static final double IntakeAngle = 75;
  public static final double PurgeAngle = 40;

  public static double tiltkP = 0.012;
  public static double tiltkI = 0.00005;
  public static double tiltkD = 0;
  public static double tiltkIz = 2.5;
  public static double tiltkFF = 0;
  public static double tiltkMinOutput = -0.5;
  public static double tiltkMaxOutput = 0.5;

  //Intake Constants
  public static int intakeMotorPort = 10;
  public static int breakBeam1Port = 0;
  public static int intakeMotorCurrentLimit = 40;
  public static double intakeSpeed = 0.8;

  //Shooter Constants
  public static int shooterMotorLeadID = 1;
  public static int shooterMotorFollowID = 2;

  public static final int ShooterCurrentLimit = 60;

  public static double shooterkp = 0;
  public static double shooterki = 0;
  public static double shooterkd = 0;
  public static double shooterks = 0;
  public static double shooterkv = 0;
  public static double shooterka = 0;


  public static final InterpolatingDoubleTreeMap distanceToAngle =
        new InterpolatingDoubleTreeMap();

    static {
      distanceToAngle.put(0.0, 105.0);
      distanceToAngle.put(1.27, 105.0);
      distanceToAngle.put(1.98, 74.0);
      distanceToAngle.put(2.74, 58.7);
      distanceToAngle.put(3.84, 47.0);
      distanceToAngle.put(5.0, 41.5);
      distanceToAngle.put(7.0, 40.0);
    }

}
